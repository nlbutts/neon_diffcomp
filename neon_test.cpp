#include <iostream>
#include <vector>
#include <fstream>
#include <arm_neon.h>
#include "timeit.h"

using namespace std;

vector<uint16_t> loadImage(const string &path)
{
    ifstream ifs(path, ios::binary | ios::ate);
    if (!ifs) {
        cerr << "Failed to open file: " << path << endl;
        return {};
    }
    streamsize size = ifs.tellg();
    ifs.seekg(0, ios::beg);

    vector<uint16_t> buffer(size / sizeof(uint16_t));
    if (!ifs.read(reinterpret_cast<char*>(buffer.data()), size)) {
        cerr << "Failed to read file: " << path << endl;
        return {};
    }
    return buffer;
}

void print_img(const vector<uint16_t> &img, int width, int height)
{
    for (int y = 0; y < height; y++) {
        cout << "Row " << y << ": ";
        for (int x = 0; x < width; x++) {
            cout << img[y * width + x] << " ";
        }
        cout << endl;
    }
}

static int16x8_t zero = vdupq_n_s16(0);
static int16x8_t ones = vdupq_n_s16(1);

int getRiceBits( unsigned short x )
{
    if (x == 0)
        return 0;
    return 32 - __builtin_clz(x);
}

void writeBitstream(uint64_t bits, int bit_count, uint8_t *bitstream, int *bitstream_size)
{
    int bit_offset = *bitstream_size;
    while (bit_count > 0) {
        int byte_offset = bit_offset / 8;
        int bit_in_byte = bit_offset % 8;
        int bits_free = 8 - bit_in_byte;
        int bits_to_write = (bit_count < bits_free) ? bit_count : bits_free;

        // Prepare mask for the bits to write
        uint8_t mask = ((1 << bits_to_write) - 1) << (bits_free - bits_to_write);

        // Shift bits to align with the current byte position
        uint8_t bits_shifted = (bits >> (bit_count - bits_to_write)) << (bits_free - bits_to_write);

        // Insert bits into the byte
        bitstream[byte_offset] &= ~mask;         // Clear the bits we're about to set
        bitstream[byte_offset] |= bits_shifted;  // Set the new bits

        bit_offset += bits_to_write;
        bit_count -= bits_to_write;
    }
    *bitstream_size = bit_offset;
}

int rice_compress(int16x8_t values, int k, uint8_t *bitstream, int *bitstream_size)
{
  // Add one to negative numbers and shift left by one
  uint16x8_t neg_mask = vcltq_s16(values, zero) & 1; // mask: 0xFFFF where negative
  uint16x8_t add_one = vaddq_u16((uint16x8_t)vabsq_s16(values), neg_mask);
  uint16x8_t shifted = vshlq_n_u16(add_one, 1);

  // To get the even numbers, we would shift left by 1, then we are shifted right, we can combine that into one opcode
  // Compute quotient: q = value >> k
  uint16x8_t quotient = vshrq_n_u16(shifted, k);

  // Compute remainder: r = value & ((1 << k) - 1)
  uint16x8_t mask = vdupq_n_u16((1 << k) - 1);
  uint16x8_t remainder = vandq_u16(shifted, mask);

  uint32x4_t ones_buffer = vdupq_n_u32(1);
  uint32x4_t q_lo = vmovl_u16(vget_low_u16(quotient));
  uint32x4_t q_hi = vmovl_u16(vget_high_u16(quotient));
  uint32x4_t bit_buffer[2];
  uint16x8_t bit_count = vdupq_n_u16(0);
  /*
  Do this
    bits <<= q + 1;
    bits |= ((1 << q) - 1) < 1;
    bit_count += q + 1;
    We may have values that overflow and do bad things, but that should be the exception rather than the norm
  */
  bit_buffer[0] = vshlq_u32(ones_buffer, (int32x4_t)q_lo);
  bit_buffer[1] = vshlq_u32(ones_buffer, (int32x4_t)q_hi);
  bit_buffer[0] = vsubq_u32(bit_buffer[0], ones_buffer);
  bit_buffer[1] = vsubq_u32(bit_buffer[1], ones_buffer);
  bit_buffer[0] = vshlq_u32(bit_buffer[0], (int32x4_t)ones_buffer);
  bit_buffer[1] = vshlq_u32(bit_buffer[1], (int32x4_t)ones_buffer);
  bit_count = vaddq_u16(bit_count, quotient);
  bit_count = vaddq_u16(bit_count, vdupq_n_u16(k + 1));

  /*
  Now do this
    bits <<= k;
    bits |= r;
    bit_count += k;
  */
  bit_buffer[0] = vshlq_u32(bit_buffer[0], vdupq_n_s32(k));
  bit_buffer[1] = vshlq_u32(bit_buffer[1], vdupq_n_s32(k));
  bit_buffer[0] = vorrq_u32(bit_buffer[0], vmovl_u16(vget_low_u16(remainder)));
  bit_buffer[1] = vorrq_u32(bit_buffer[1], vmovl_u16(vget_high_u16(remainder)));

  for (int i = 0; i < 8; i++) {
    int16_t q = quotient[i];
    int16_t r = remainder[i];

    if (q > 8)
    {
      uint64_t bits = 0;
      bits |= 0xFF;
      q -= 8;
      int o = getRiceBits(q);
      bits <<= o;
      bits |= ((1 << o) - 1);
      bits <<= 1;

      bits <<= o - 1;
      bits |= (q & ((1 << (o - 1)) - 1));

      bits <<= k;
      bits |= r;
      int temp_bit_count = 8 + o + 1 + (o - 1) + k;
      writeBitstream(bits, temp_bit_count, bitstream, bitstream_size);
    }
    else
    {
      // Just write the bitstream
      uint64_t bits = bit_buffer[i >> 2][i & 0x3];
      int temp_bit_count = bit_count[i];
      writeBitstream(bits, temp_bit_count, bitstream, bitstream_size);
    }
  }

  return 0;
}

vector<uint8_t> diffcomp(vector<uint16_t> &input, int width, int height)
{
  vector<uint8_t> even_output(input.size(), 0);
  vector<uint8_t> odd_output(input.size(), 0);
  int k = 7;
  int16_t * src = (int16_t*)input.data();

  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col += 16)
    {
      int16x8x2_t linedata = vld2q_s16(src);
      src += 16;

      // Subtract one channel from the other
      int16x8_t even = linedata.val[0];
      int16x8_t odd = linedata.val[1];
      int16x8_t diff = even - odd;

      // Compute the difference between each element and the previous one
      int16x8_t shifted_even = vextq_s16(zero, even, 7);
      int16x8_t shifted_diff = vextq_s16(zero, diff, 7);
      int16x8_t even_diff = vsubq_s16(even, shifted_even);
      int16x8_t diff_diff = vsubq_s16(odd, shifted_diff);

      int even_bit_count = 0;
      int odd_bit_count = 0;

      rice_compress(even_diff, k, even_output.data(), &even_bit_count);
      rice_compress(diff_diff, k, odd_output.data(), &odd_bit_count);
    }
  }

  return vector<uint8_t>();
}

int main()
{
  int width = 1920;
  int height = 1080;
  //cout << "ARM Intrinsic test" << endl;
  auto image = loadImage("bayer.raw");
  //print_img(image, width, height);
  //cout << "Image size: " << image.size() << " pixels" << endl;
  {
    Timeit t("diffcomp");
    auto output = diffcomp(image, width, height);
  }
}

