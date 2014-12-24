
static int ubus_uart_init(char *uart_device, int baud_rate)
{
    struct termios newtio;
    int fd;

    fd = open(uart_device, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd < 0) {
        LOG_ERR("unable to init uart port: %s\n", strerror(errno));
        return -1;
    }

    if (fcntl(fd, F_SETFL, 0) < 0) {
        LOG_ERR("fcntl failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

//    tcflush(fd, TCIOFLUSH);
    if (tcgetattr(fd, &newtio) < 0) {
        LOG_ERR("tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    oldtio = newtio;

#if 1
    cfmakeraw(&newtio);
    switch (baud_rate) {
    case 9600:
        cfsetispeed(&newtio, B9600);
        //cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        //cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        //cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetispeed(&newtio, B57600);
        //cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        //cfsetospeed(&newtio, B115200);
        break;
    default:
        break;
    }

    newtio.c_cc[VTIME] = 10;     /* inter-character timer in deciseconds */
    newtio.c_cc[VMIN] = 0;      /* blocking read until n chars received */

    //newtio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
    newtio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
    //newtio.c_cflag |= CS8 | CLOCAL;
    newtio.c_cflag |= CS8;

    //newtio.c_iflag &= ~(IXON | IXOFF |IXANY);
    //newtio.c_iflag |= IXANY;
    if(newtio.c_iflag | (IXON)) {
        fprintf(stderr, "IXON...\n");
    }
    if(newtio.c_iflag | (IXOFF)) {
        fprintf(stderr, "IXOFF...\n");
    }
    if(newtio.c_iflag | (IXANY)) {
        fprintf(stderr, "IXANY...\n");
    }

//    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) < 0) {
        LOG_ERR("tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
#else
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 1;     /* inter-character timer unused */
    newtio.c_cc[VMIN] = 1;      /* blocking read until 5 chars received */

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) < 0) {
        LOG_ERR("tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
#endif
    return fd;
}
