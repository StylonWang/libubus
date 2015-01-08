#ifndef __TEST_MULTI_PIPE_H__
#define __TEST_MULTI_PIPE_H__

// common parameter used by multi pipe testing

#define MAX_TEST_PIPES (4)

static struct {
    packet_sig_t request_sig;
    packet_sig_t reply_sig;
} test_signatures[MAX_TEST_PIPES+1] = {
    { { 0xa0, 0xa1, 0xa2, }, { 0xa3, 0xa4, 0xa5}, }, // for pipe 1
    { { 0xa6, 0xa7, 0xa8, }, { 0xa9, 0xaa, 0xab}, }, // for pipe 2
    { { 0xac, 0xad, 0xae, }, { 0xaf, 0xb0, 0xb1}, }, // for pipe 3
    { { 0xb2, 0xb3, 0xb4, }, { 0xb5, 0xb6, 0xb7}, }, // for pipe 4
    { { 0xb8, 0xb9, 0xba, }, { 0xbb, 0xbc, 0xbd}, }, // for pipe 5
};


#endif //__TEST_MULTI_PIPE_H__
