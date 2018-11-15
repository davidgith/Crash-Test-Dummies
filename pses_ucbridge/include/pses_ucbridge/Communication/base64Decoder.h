/**
 * @file "pses_ucbridge/Communication/base64Decoder.h"
 * @brief This library provides functions to convert a base64 encoded value to
 *32bit signed/unsigned integer.
 *
*/

#include <vector>
#include <string>

/**
 * @brief Decodes a base64 encoded string to bytes.
 * @param[in] in input string to be decoded
 * @return byte array decoded from the input string
 * @author azawadzki - from:
 * https://stackoverflow.com/questions/180947/base64-decode-snippet-in-c
*/
inline std::vector<unsigned char> base64_to_binary(const std::string& in)
{

  std::string out;
  std::string in_padded;

  // Add padding
  if (in_padded.length() % 4 == 2)
  {
    in_padded = in + "==";
  }

  else if (in_padded.length() % 4 == 3)
  {
    in_padded = in + "=";
  }

  else
  {
    in_padded = in;
  }

  std::vector<int> T(256, -1);
  for (int i = 0; i < 64; i++)
    T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] =
        i;

  int val = 0, valb = -8;
  for (unsigned char c : in_padded)
  {
    if (T[c] == -1)
      break;
    val = (val << 6) + T[c];
    valb += 6;
    if (valb >= 0)
    {
      out.push_back(char((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return std::vector<unsigned char>(out.begin(), out.end());
}

/**
 * @brief Concatenates bytes from index a to index b of a byte array
 * @param[in] bytes byte array
 * @param[in] startByteIndex start index a
 * @param[in] endByteIndex end index b
 * @return unsigned 64bit integer containing the concatenated bytes
*/
inline unsigned long getValueAt(std::vector<unsigned char>& bytes,
                                const unsigned int startByteIndex,
                                const unsigned int endByteIndex)
{
  unsigned long out = 0;

  for (int i = startByteIndex; i <= endByteIndex; i++)
  {
    if (i >= bytes.size())
      throw std::out_of_range("Out of bounds error while decoding b64.");
    out = out | ((0x0000000000000000 | bytes[i]) << 8 * (i - startByteIndex));
  }
  return out;
}

/**
 * @brief Converts a 32bit signed/unsigned integer from its bit representation
 * to signed 64bit integer.
 * @param[in] value input number encoded in 64 bits
 * @param[in] size byte size the output is supposed to have
 * @param[in] isSigned should the output be signed or unsigned?
 * @return output signed/unsigned 32bit integer inside a signed 64bit integer
*/
inline long convertValue(const unsigned long value, const unsigned int size,
                         bool isSigned)
{
  if (!isSigned)
    return static_cast<unsigned int>(value);
  unsigned long mask = 0x0000000000000080;
  mask = mask << (8 * (size - 1));
  mask = mask & value;
  for (int i = 0; i < 64 - (size * 8); i++)
  {
    mask = mask | (mask << 1);
  }
  return value | mask;
}

/**
 * @brief Function decodes a Base64 string and returns the decoded string (or
 *only
 * parts of it) up to a maximum of 8 byte signed/unsigned data types
 * wrapped in an int64 return value.
 *
 * This Fuction decodes a Base64 string and returns a specific byte or multiple
 * bytes at the given indices,
 * where this information is converted as a desired datatype and returned as
 * int64.
 * Example for decoding a Base64 string, getting the second byte, reading this
 *value as an int16:
 * base64_decode("Afb/", 1, 2, 16, true).
 *
 * @param[in] in input Base64 string.
 * @param[in] startByteIndex index number (starting with zero) of the first byte
 *of
 *the value that has to be read.
 * @param[in] endByteIndex index number (starting with zero) of the last byte of
 *the
 *value that has to be read.
 * @param[in] size size in bits of the value that is to be read from the decoded
 *string.
 * @param[in] isSigned whether the value from the decoded string is to be read
 *as
 *signed or unsigned.
 * @return Decoded information as an int64.
*/
inline long base64_decode(const std::string& in,
                          const unsigned int startByteIndex,
                          const unsigned int endByteIndex,
                          const unsigned int size, bool isSigned)
{
  std::vector<unsigned char> bytes = base64_to_binary(in);
  unsigned long value = getValueAt(bytes, startByteIndex, endByteIndex);
  return convertValue(value, size, isSigned);
}
