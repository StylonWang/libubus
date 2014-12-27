#ifndef __TEST_CASE_H__
#define __TEST_CASE_H__

static ubus_request_t test_cases[] = {
    // command, data, data length
    { 0x01, {0x01, }, 0x01 },
    { 0x02, {0x01, }, 0x01 },
    { 0x03, {0x01, }, 0x01 },
};


#endif //__TEST_CASE_H__
