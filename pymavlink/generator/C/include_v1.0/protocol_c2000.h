#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file protocol_c2000.h
 * @author Aaron Bamberger (abamberger@aesaustin.com)
 * @date December, 2014
 * @brief Data packing, unpacking, and CRC functions for the TI C2000 architecture
 *
 * The C2000 architecture doesn't have byte-level addressing, only (16-bit) word level addressing,
 * which means, in addition to other things, the char and uint8_t types are actually 16-bits wide.
 * This necessitates special handling of all data packing and unpacking into and out of mavlink_message_t
 * payloads, as well as special consideration when calculating CRCs.  This file contains support functions
 * that replace some of the normal MAVLink library support functions when compiling for the C2000 architecture,
 * which do the necessary work to unpack and repack message payloads and calculate CRCs while handing the
 * peculiarities of the C2000 architecture
 */

#ifndef PROTOCOL_C2000_H_
#define PROTOCOL_C2000_H_

#include <string.h>

#define X25_INIT_CRC_C2000 0xffff

/**
 * @brief Insert a 32-bit floating point number into the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be inserted
 * @param b Value to be inserted into the payload
 *
 */
static inline void mav_put_float_c2000(void* buf, int wire_offset, float b)
{
    float* dest_ptr = (float*)((uint8_t*)buf + (wire_offset / 2));
    *dest_ptr = b;
}

/**
 * @brief Insert a 32-bit integer into the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be inserted
 * @param b Value to be inserted into the payload
 *
 */
static inline void mav_put_uint32_t_c2000(void* buf, int wire_offset, uint32_t b)
{
    uint32_t* dest_ptr = (uint32_t*)((uint8_t*)buf + (wire_offset / 2));
    *dest_ptr = b;
}

/**
 * @brief Insert a 16-bit integer into the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be inserted
 * @param b Value to be inserted into the payload
 *
 */
static inline void mav_put_uint16_t_c2000(void* buf, int wire_offset, uint16_t b)
{
    uint16_t* dest_ptr = (uint16_t*)((uint8_t*)buf + (wire_offset / 2));
    *dest_ptr = b;
}

/**
 * @brief Insert an 8-bit integer into the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be inserted
 * @param b Value to be inserted into the payload
 *
 */
static inline void mav_put_uint8_t_c2000(void* buf, int wire_offset, uint8_t b)
{
    uint8_t* dest_ptr = (uint8_t*)buf;
    if ((wire_offset % 2) == 0) {
        dest_ptr += (wire_offset / 2);
        *dest_ptr &= 0xFF00;
        *dest_ptr |= (b & 0x00FF);
    } else {
        dest_ptr += ((wire_offset - 1) / 2);
        *dest_ptr &= 0x00FF;
        *dest_ptr |= ((b << 8) & 0xFF00);
    }
}

/**
 * @brief Copy an array of floats into the payload of a message for the C2000 architecture
 *
 * @param buf_dest Pointer to beginning of MAVLink message payload
 * @param buf_src Pointer to beginning of array to be copied
 * @param wire_offset Offset in bytes into the payload where the array should be copied
 * @param len Length of the array to be copied in array elements (not bytes)
 *
 */
static inline void mav_put_float_array_c2000(void* buf_dest, const float* buf_src, int wire_offset, int len)
{
    int i;
    if (buf_src == NULL) {
        for (i = 0; i < len; i++) {
            mav_put_float_c2000(buf_dest, wire_offset + i, 0.0);
        }
    } else {
        for (i = 0; i < len; i++) {
            mav_put_float_c2000(buf_dest, wire_offset + i, buf_src[i]);
        }
    }
}

/**
 * @brief Copy an array of 32-bit integers into the payload of a message for the C2000 architecture
 *
 * @param buf_dest Pointer to beginning of MAVLink message payload
 * @param buf_src Pointer to beginning of array to be copied
 * @param wire_offset Offset in bytes into the payload where the array should be copied
 * @param len Length of the array to be copied in array elements (not bytes)
 *
 */
static inline void mav_put_uint32_t_array_c2000(void* buf_dest, const uint32_t* buf_src, int wire_offset, int len)
{
    int i;
    if (buf_src == NULL) {
        for (i = 0; i < len; i++) {
            mav_put_uint32_t_c2000(buf_dest, wire_offset + i, 0x00000000);
        }
    } else {
        for (i = 0; i < len; i++) {
            mav_put_uint32_t_c2000(buf_dest, wire_offset + i, buf_src[i]);
        }
    }
}


/**
 * @brief Copy an array of 16-bit integers into the payload of a message for the C2000 architecture
 *
 * @param buf_dest Pointer to beginning of MAVLink message payload
 * @param buf_src Pointer to beginning of array to be copied
 * @param wire_offset Offset in bytes into the payload where the array should be copied
 * @param len Length of the array to be copied in array elements (not bytes)
 *
 */
static inline void mav_put_uint16_t_array_c2000(void* buf_dest, const uint16_t* buf_src, int wire_offset, int len)
{
    int i;
    if (buf_src == NULL) {
        for (i = 0; i < len; i++) {
            mav_put_uint16_t_c2000(buf_dest, wire_offset + i, 0x0000);
        }
    } else {
        for (i = 0; i < len; i++) {
            mav_put_uint16_t_c2000(buf_dest, wire_offset + i, buf_src[i]);
        }
    }
}

/**
 * @brief Copy an array of 8-bit integers into the payload of a message for the C2000 architecture
 *
 * @param buf_dest Pointer to beginning of MAVLink message payload
 * @param buf_src Pointer to beginning of array to be copied
 * @param wire_offset Offset in bytes into the payload where the array should be copied
 * @param len Length of the array to be copied in array elements (not bytes)
 *
 */
static inline void mav_put_char_array_c2000(void* buf_dest, const char* buf_src, int wire_offset, int len)
{
    int i;
    if (buf_src == NULL) {
        for (i = 0; i < len; i++) {
            mav_put_uint8_t_c2000(buf_dest, wire_offset + i, 0x0000);
        }
    } else {
        for (i = 0; i < len; i++) {
            mav_put_uint8_t_c2000(buf_dest, wire_offset + i, buf_src[i]);
        }
    }
}

/**
 * @brief Extract a float from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be extracted from
 * @return Value extracted from message payload
 */
static inline float mav_get_float_c2000(const void* buf, int wire_offset)
{
    float* dest_ptr = (float*)((uint8_t*)buf + (wire_offset / 2));
    return *dest_ptr;
}

/**
 * @brief Extract a 32-bit integer from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be extracted from
 * @return Value extracted from message payload
 */
static inline uint32_t mav_get_uint32_t_c2000(const void* buf, int wire_offset)
{
    uint32_t* dest_ptr = (uint32_t*)((uint8_t*)buf + (wire_offset / 2));
    return *dest_ptr;
}

/**
 * @brief Extract a 16-bit integer from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be extracted from
 * @return Value extracted from message payload
 */
static inline uint16_t mav_get_uint16_t_c2000(const void* buf, int wire_offset)
{
    uint16_t* dest_ptr = (uint16_t*)((uint8_t*)buf + (wire_offset / 2));
    return *dest_ptr;
}

/**
 * @brief Extract an 8-bit integer from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param wire_offset Offset in bytes into the payload where the value should be extracted from
 * @return Value extracted from message payload
 */
static inline uint8_t mav_get_uint8_t_c2000(const void* buf, int wire_offset)
{
    uint8_t* dest_ptr = (uint8_t*)buf;
    if ((wire_offset % 2) == 0) {
        dest_ptr += (wire_offset / 2);
        return (*dest_ptr & 0x00FF);
    } else {
        dest_ptr += ((wire_offset - 1) / 2);
        return ((*dest_ptr >> 8) & 0x00FF);
    }
}

/**
 * @brief Extract an array of floats from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param value Pointer to destination array to copy extracted array into
 * @param array_length Length of array to extract in array elements (not bytes)
 * @param wire_offset Offset in bytes into the payload where the array should be extracted from
 * @return Length of the extracted array
 */
static inline uint16_t mav_get_float_array_c2000(const void* buf, float* value, int array_length, int wire_offset)
{
    int i;
    for (i = 0; i < array_length; i++) {
        value[i] = mav_get_float_c2000(buf, wire_offset + i);
    }

    return array_length;
}

/**
 * @brief Extract an array of 32-bit integers from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param value Pointer to destination array to copy extracted array into
 * @param array_length Length of array to extract in array elements (not bytes)
 * @param wire_offset Offset in bytes into the payload where the array should be extracted from
 * @return Length of the extracted array
 */
static inline uint16_t mav_get_uint32_t_array_c2000(const void* buf, uint32_t* value, int array_length, int wire_offset)
{
    int i;
    for (i = 0; i < array_length; i++) {
        value[i] = mav_get_uint32_t_c2000(buf, wire_offset + i);
    }

    return array_length;
}

/**
 * @brief Extract an array of 16-bit integers from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param value Pointer to destination array to copy extracted array into
 * @param array_length Length of array to extract in array elements (not bytes)
 * @param wire_offset Offset in bytes into the payload where the array should be extracted from
 * @return Length of the extracted array
 */
static inline uint16_t mav_get_uint16_t_array_c2000(const void* buf, uint16_t* value, int array_length, int wire_offset)
{
    int i;
    for (i = 0; i < array_length; i++) {
        value[i] = mav_get_uint16_t_c2000(buf, wire_offset + i);
    }

    return array_length;
}

/**
 * @brief Extract an array of 8-bit integers from the payload of a message for the C2000 architecture
 *
 * @param buf Pointer to beginning of MAVLink message payload
 * @param value Pointer to destination array to copy extracted array into
 * @param array_length Length of array to extract in array elements (not bytes)
 * @param wire_offset Offset in bytes into the payload where the array should be extracted from
 * @return Length of the extracted array
 */
static inline uint16_t mav_get_char_array_c2000(const void* buf, char* value, int array_length, int wire_offset)
{
    int i;
    for (i = 0; i < array_length; i++) {
        value[i] = mav_get_uint8_t_c2000(buf, wire_offset + i);
    }

    return array_length;
}

/**
 * @brief Initiliaze the buffer for the X.25 CRC
 *
 * This is identical to the non-C2000 version of this function (the constant used is also identical)
 * It is simply replicated here to remove a circular dependency issue between this header and checksum.h
 *
 * @param crcAccum the 16 bit X.25 CRC
 */
static inline void crc_init_c2000(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC_C2000;
}


/**
 * @brief Accumulate the X.25 CRC by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * The C2000 version is unique in that it needs to do extra masking of the incoming data byte
 * and temporary accumulator byte, since uint8_t on C2000 is actually 16-bits.  Without doing
 * the extra masking rollover into the upper byte of the word causes the CRC to be calculated
 * incorrectly
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 */
static inline void crc_accumulate_c2000(uint8_t data, uint16_t *crcAccum)
{
    /*Accumulate one byte of data into the CRC*/
    uint8_t tmp = 0x0000;

    // For C2000, since bytes are actually stored in the lower half of a 16-bit word,
    // need to make sure we mask off the upper half of the word so we don't accidentally accumulate
    // unwanted garbage data into the CRC
    tmp = (data & 0x00FF) ^ (uint8_t)(*crcAccum & 0x00ff);
    tmp ^= (tmp << 4);
    tmp &= 0x00FF; // Need to zero-out the upper byte of tmp, since it's intended to be an 8-bit value (but is actually 16-bits),
                   // and rolling it back into the 16-bit crcAccum with garbage in the upper byte will corrupt the CRC
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

/**
 * @brief Calculates the X.25 checksum on a byte buffer
 *
 * The only difference between this and the non-C2000 version is that this calls the C2000 specific
 * version of crc_accumulate to make sure that 16-bit "bytes" are handled correctly
 *
 * @param  pBuffer buffer containing the byte array to hash
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 */
static inline uint16_t crc_calculate_c2000(const uint8_t* pBuffer, uint16_t length)
{
    uint16_t crcTmp;
    crc_init_c2000(&crcTmp);
    while (length--) {
        crc_accumulate_c2000(*pBuffer++, &crcTmp);
    }
    return crcTmp;
}

/**
 * @brief Accumulate the X.25 CRC by adding an array of bytes
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * The C2000 version is specific to message payloads (where the normal version works for any byte buffer),
 * but in the library this is the only way the normal version is used.  The C2000 version uses the C2000-specific
 * byte retrieval function to handle extracting bytes out of a message payload on the C2000 architecture
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 */
static inline void crc_accumulate_msg_payload_c2000(uint16_t *crcAccum, void* payload, uint16_t length)
{
    int crc_bytes_accumulated = 0;
    while (crc_bytes_accumulated < length) {
        crc_accumulate_c2000(mav_get_uint8_t_c2000(payload, crc_bytes_accumulated++), crcAccum);
    }
}

#endif /* PROTOCOL_C2000_H_ */

#ifdef __cplusplus
}
#endif
