
/*
 * Copyright (c) 2016-2017 Denny Page
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <assert.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/ethtool.h>
#include <linux/net_tstamp.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/ptp_clock.h>


/*
 * Notes:
 *
 * Hardware timesamps are based on the last bit of the SFD for both transmit and receive.
 *
 * While the interpacket gap is commonly thought of as following the packet, it's actually
 * a carrier sense preceeding the packet. As such, it's part of standard switch forwarding.
 * Technically, a switch (particularly a cut-though switch) could eliminate the latency of
 * the carrier sense on an idle connection by continually listening for silence even when
 * it has no frame to transmit, but this does not seem to be the case. The inclusion of
 * ETHER_CS_BITS in ETHER_CTSW_RX_BITS may need to be revisited in the future.
 *
 * In-between creation of the socket and binding it to a specific interface, the kernel
 * will queued packets from all interfaces to the socket.
 */


// Ethernet info
#define ETHER_PROTO             (46)
#define ETHER_PACKET_BYTES      (60L)           // Includes header, but not preabmle/sfd/fcs
#define ETHER_CS_BITS           (96)            // Carrier sense bits (AKA Inter Packet Gap)
#define ETHER_PREAMBLE_BITS     (64L)           // Includes SFD
#define ETHER_MAC_BITS          (48L)           // One MAC address
#define ETHER_FCS_BITS          (32L)           // Frame Check Sequence
#define ETHER_PACKET_BITS       (ETHER_PACKET_BYTES * 8L)

#define ETHER_SFSW_TX_BITS      (ETHER_PACKET_BITS + ETHER_FCS_BITS)
#define ETHER_SFSW_RX_BITS      (ETHER_CS_BITS + ETHER_PREAMBLE_BITS)

#define ETHER_CTSW_TX_BITS      (ETHER_MAC_BITS)
#define ETHER_CTSW_RX_BITS      (ETHER_CS_BITS + ETHER_PREAMBLE_BITS)

#define NSEC_PER_METER          (5.0)
#define MILLION                 (1000000L)
#define BILLION                 (1000000000L)

#define START_DELAY             (1000L)         // Microseconds to delay before starting
#define TX_SEND_DELAY           (100L)          // Microseconds to delay before each send
#define WARNING_DELAY           (500L * 1000L)  // Microseconds to delay for warning

#define TX_POLL_LIMIT           (1000L)         // Milliseconds to wait for timestamp
#define RX_POLL_LIMIT           (2000L)         // Milliseconds to wait for packet

#define CMSG_BUF_LEN            (256)


// Command line parameters
static const char *             progname;
static char *                   data_file_name = NULL;
static FILE *                   data_file = NULL;
static unsigned int             hwstamps = 1;
static unsigned int             iterations = 1000;
static unsigned int             ptp_samples = PTP_MAX_SAMPLES;
static long                     rx_comp = 0;
static long                     tx_comp = 0;
static long                     sw_comp = 0;
static double                   cable_len = 0.0;

// tx/rx sockets
static int                      tx_socket;
static int                      rx_socket;

// tx/rx interface information
static char *                   tx_name;
static char *                   rx_name;
static int                      tx_index;
static int                      rx_index;
static struct ether_addr        tx_addr;
static struct ether_addr        rx_addr;
static int                      tx_speed;
static int                      rx_speed;

// ptp clock info
static int                      tx_ptpdev;
static int                      rx_ptpdev;
static int                      tx_ptp_index;
static int                      rx_ptp_index;

// tx packet buffer
static char                     tx_packet[ETHER_PACKET_BYTES];

// tx and rx info structures
typedef struct
{
    struct timespec             timestamp;
    long                        before_send_to_after_send;
    long                        before_send_to_timestamp;
    long                        after_send_to_before_poll;
    long                        after_send_to_after_poll;
}
tx_info;

typedef struct
{
    struct timespec             timestamp;
    long                        timestamp_to_poll_return;
}
rx_info;

static tx_info *                tx_values;
static rx_info *                rx_values;



//
// Error
//
__attribute__ ((noreturn, format (printf, 1, 2)))
static void
fatal(
    const char *                format,
    ...)
{
    va_list                     args;

    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);

    exit(EXIT_FAILURE);
}



//
// Delay for N microseconds
//
static void
delay(
    unsigned long               usec)
{
    struct timespec             ts;
    int                         r;

    ts.tv_sec = usec / MILLION;
    ts.tv_nsec = usec % MILLION * 1000L;
    r = nanosleep(&ts, NULL);
    if (r == -1)
    {
        perror("nanosleep");
        fatal("nanosleep failed");
    }
}



//
// Create a socket on an interface and set for raw ethernet
//
static int
socket_create(
    const char *                ifname,
    int *                       index,
    int *                       phc_index,
    struct ether_addr *         addr,
    int *                       speed)
{
    struct sockaddr_ll          sockaddr_ll;
    struct ifreq                iface;
    struct ethtool_cmd          ether_cmd;
    struct ethtool_ts_info      ether_info;
    struct ethtool_eee          ether_eee;
    struct ethtool_coalesce     ether_coalesce;
    struct hwtstamp_config      tsconfig;
    int                         sock;
    int                         flags;
    int                         r;

    // Create the socket
    sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sock == -1)
    {
        perror("socket");
        fatal("cannot create socket\n");
    }

    // Get the interface index
    memset(&iface, 0, sizeof(iface));
    strncpy((char *) iface.ifr_name, ifname, sizeof(iface.ifr_name) - 1);
    r = ioctl(sock, SIOCGIFINDEX, &iface);
    if (r == -1)
    {
        perror("ioctl");
        fatal("request of interface index for %s failed\n", ifname);
    }
    *index = iface.ifr_ifindex;

    // Bind the socket
    memset(&sockaddr_ll, 0, sizeof(sockaddr_ll));
    sockaddr_ll.sll_family = AF_PACKET;
    sockaddr_ll.sll_protocol = htons(ETH_P_ALL);
    sockaddr_ll.sll_ifindex = iface.ifr_ifindex;
    sockaddr_ll.sll_halen = 1;
    // Valgrind reports that bind reads beyond the end of the sockaddr_ll. There doesn't seem
    // to be a reasonable way to prevent this.
    r = bind(sock, (struct sockaddr *) &sockaddr_ll, sizeof(sockaddr_ll));
    if (r == -1)
    {
        perror("bind");
        fatal("request to bind socket to interface %s failed\n", ifname);
    }

    // Get hardware (mac) address of interface
    r = ioctl(sock, SIOCGIFHWADDR, &iface);
    if (r == -1)
    {
        perror("ioctl");
        fatal("request for hardware address of %s failed\n", ifname);
    }
    if (iface.ifr_ifru.ifru_hwaddr.sa_family != ARPHRD_ETHER)
    {
        fatal("request for hardware address of %s returned something other than ethernet\n", ifname);
    }
    memcpy(addr, iface.ifr_ifru.ifru_hwaddr.sa_data, sizeof(*addr));

    // Get interface speed
    //
    //   NB: ETHTOOL_GLINKSETTINGS is now preferred, but this requires Linux 4.4. Stay with
    //       the older command for now.
    //
    memset(&ether_cmd, 0, sizeof(ether_cmd));
    ether_cmd.cmd = ETHTOOL_GSET;
    iface.ifr_data = (void *)&ether_cmd;
    r = ioctl(sock, SIOCETHTOOL, &iface);
    if (r == -1)
    {
        perror("ioctl");
        fatal("request iterface configuration for %s failed\n", ifname);
    }
    *speed = (int) ethtool_cmd_speed(&ether_cmd);
    if (*speed == 0 || *speed == SPEED_UNKNOWN)
    {
        fatal("interface speed for %s is unkown\n", ifname);
    }

    // Check for eee
    memset(&ether_eee, 0, sizeof(ether_eee));
    ether_eee.cmd =  ETHTOOL_GEEE;
    iface.ifr_data = (void *)&ether_eee;
    r = ioctl(sock, SIOCETHTOOL, &iface);
    if (r == -1)
    {
        perror("ioctl");
        fatal("request eee configuration for %s failed\n", ifname);
    }
    if (ether_eee.eee_active == 1)
    {
        printf("WARNING: Energy Efficient Ethernet is active on interface %s. Results will be incorrect.\n\n", ifname);
        fflush(stdout);
        delay(WARNING_DELAY);
    }

    // Check for interrupt coalescing
    memset(&ether_coalesce, 0, sizeof(ether_coalesce));
    ether_coalesce.cmd =  ETHTOOL_GCOALESCE;
    iface.ifr_data = (void *)&ether_coalesce;
    r = ioctl(sock, SIOCETHTOOL, &iface);
    if (r == -1)
    {
        perror("ioctl");
        fatal("request eee configuration for %s failed\n", ifname);
    }
    if (ether_coalesce.tx_coalesce_usecs || ether_coalesce.tx_coalesce_usecs_irq ||
        ether_coalesce.rx_coalesce_usecs || ether_coalesce.rx_coalesce_usecs_irq)
    {
        printf("WARNING: Interrupt coalescing is active on interface %s. Results will be inaccurate.\n\n", ifname);
        fflush(stdout);
        delay(WARNING_DELAY);
    }

    // Set up timestamping
    if (hwstamps)
    {
        // Get phc (ptp) device index
        memset(&ether_info, 0, sizeof(ether_info));
        ether_info.cmd = ETHTOOL_GET_TS_INFO;
        iface.ifr_data = (void *)&ether_info;
        r = ioctl(sock, SIOCETHTOOL, &iface);
        if (r == -1)
        {
            perror("ioctl");
            fatal("request iterface phc index for %s failed\n", ifname);
        }
        *phc_index = ether_info.phc_index;

        // Enable hardware tiimestamps on the interface
        //
        //   NB: We actually could be more selective about tx/rx, but this has less chance of
        //       interferring with another process that is already using hardware timestamps
        //       such as ptp or ntp.
        //
        memset(&tsconfig, 0, sizeof(tsconfig));
        tsconfig.tx_type = HWTSTAMP_TX_ON;
        tsconfig.rx_filter = HWTSTAMP_FILTER_ALL;
        iface.ifr_data = (void *) &tsconfig;
        r = ioctl(sock, SIOCSHWTSTAMP, &iface);
        if (r == -1)
        {
            perror("ioctl");
            fatal("request to enable hardware timestamp for %s failed\n", ifname);
        }

        // Enable hardware timestamps on the socket
        flags = SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
        r = setsockopt(sock, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags));
        if (r == -1)
        {
            perror("ioctl");
            fatal("request to enable hardware timestamp on socket failed\n");
        }
    }
    else
    {
        // Enable software timestamps on the socket
        flags = SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_RX_SOFTWARE;
        r = setsockopt(sock, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags));
        if (r == -1)
        {
            perror("ioctl");
            fatal("request to enable hardware timestamp on socket failed\n");
        }
    }
    return(sock);
}



//
// Drain messages from a socket
//
static void
drain(
    int                         sock,
    int                         errorqueue)
{
    struct msghdr               msg;
    struct iovec                iovec;
    char                        packet[ETH_FRAME_LEN];
    char                        cmsg_buf[CMSG_BUF_LEN];
    long                        len;

    do
    {
        memset(&msg, 0, sizeof(msg));
        iovec.iov_base = &packet;
        iovec.iov_len = sizeof(packet);
        msg.msg_name = NULL;
        msg.msg_iov = &iovec;
        msg.msg_iovlen = 1;
        msg.msg_control = &cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);

        len = recvmsg(sock, &msg, MSG_DONTWAIT | errorqueue);
    }
    while (len > 0);
}



//
// Open the ptp control file for an interface
//
static int
ptpdev_open(
    int                         index)
{
    char                        name[256];
    int                         r;

    r = snprintf(name, sizeof(name), "/dev/ptp%d", index);
    if (r < 0)
    {
        fatal("failed to create ptp name\n");
    }

    r = open(name, O_RDONLY);
    if (r == -1)
    {
        fatal("failed to open ptp defice %s\n", name);
    }

    return(r);
}



//
// Add N nanoseconds to a timespec
//
static void
timespec_addns(
    struct timespec *           ts,
    long                        nsec)
{
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= BILLION)
    {
        ts->tv_sec += ts->tv_nsec / BILLION;
        ts->tv_nsec = ts->tv_nsec % BILLION;
    }
    else if (ts->tv_nsec < 0)
    {
        ts->tv_sec += ts->tv_nsec / BILLION - 1;
        ts->tv_nsec = ts->tv_nsec % BILLION + BILLION;
    }
}



//
// Delta between two timespecs in nanoseconds
//
static long
timespec_delta(
    const struct timespec *     ts1,
    const struct timespec *     ts2)
{
    long long                   t1_nsec;
    long long                   t2_nsec;

    t1_nsec = ts1->tv_sec * BILLION;
    if (t1_nsec >= 0)
    {
        t1_nsec += ts1->tv_nsec;
    }
    else
    {
        t1_nsec -= ts1->tv_nsec;
    }

    t2_nsec = ts2->tv_sec * BILLION;
    if (t2_nsec >= 0)
    {
        t2_nsec += ts2->tv_nsec;
    }
    else
    {
        t2_nsec -= ts2->tv_nsec;
    }

    return((long) (t2_nsec - t1_nsec));
}



//
// sqrt function for standard deviation
//
static unsigned long
llsqrt(
    unsigned long long          x)
{
    unsigned long long          prev;
    unsigned long long          s;

    s = x;
    if (s)
    {
        prev = ~((unsigned long long) 1 << 63);

        while (s < prev)
        {
            prev = s;
            s = (s + (x / s)) / 2;
        }
    }

    return (unsigned long) s;
}



//
// Determine the median of an array of longs
// (Thanks to Torben Mogensen)
//
static long
median(
    long                        array[],
    unsigned int                count)
{
    long                        min;
    long                        max;
    long                        guess;
    long                        maxltguess;
    long                        mingtguess;
    unsigned int                less;
    unsigned int                greater;
    unsigned int                equal;
    unsigned int                i;

    min = array[0];
    max = array[0];

    for (i = 1; i < count; i++)
    {
        if (array[i] < min)
        {
            min = array[i];
        }
        if (array[i] > max)
        {
            max = array[i];
        }
    }

    while (1) {
        less = 0;
        greater = 0;
        equal = 0;
        guess = (min + max) / 2;
        maxltguess = min;
        mingtguess = max;

        for (i = 0; i < count; i++)
        {
            if (array[i] < guess)
            {
                less++;
                if (array[i] > maxltguess)
                {
                    maxltguess = array[i];
                }
            }
            else if (array[i] > guess)
            {
                greater++;
                if (array[i] < mingtguess)
                {
                    mingtguess = array[i];
                }
            }
            else
            {
                equal++;
            }
        }
        if (less <= (count + 1) / 2 && greater <= (count + 1) / 2)
        {
            break;
        }
        else if (less > greater)
        {
            max = maxltguess;
        }
        else
        {
            min = mingtguess;
        }
    }

    if (less >= (count + 1) / 2) {
        return(maxltguess);
    }
    else if (less + equal >= (count + 1) / 2)
    {
        return(guess);
    }
    else
    {
        return(mingtguess);
    }
}



//
// Capture the current ptp/sys clock offset
//
static void
ptp_offset_capture(
    int                         ptpdev,
    struct ptp_sys_offset *     ptp_offset)
{
    int                         r;

    ptp_offset->n_samples = (unsigned int) ptp_samples;

    r = ioctl(ptpdev, PTP_SYS_OFFSET, ptp_offset);
    if (r == -1)
    {
        perror("ioctl");
        fatal("request for PTP_SYS_OFFSET failed\n");
    }
}



#if 1

//
// Choose the best ptp/sys clock offset by the narrowest window
//
static long
best_ptp_offset(
    struct ptp_sys_offset *     ptp_offset)
{
    struct ptp_clock_time *     pct;
    long long                   sclock1, sclock2, pclock;
    long                        window, best_window;
    long long                   best_sclock, best_pclock;
    unsigned int                i;

    pct = ptp_offset->ts;

    sclock1 = pct[0].sec * BILLION + pct[0].nsec;
    pclock = pct[1].sec * BILLION + pct[1].nsec;
    sclock2 = pct[2].sec * BILLION + pct[2].nsec;

    best_window = sclock2 - sclock1;
    best_sclock = sclock1;
    best_pclock = pclock;

    for (i = 1; i < ptp_offset->n_samples; i++)
    {
        pct = ptp_offset->ts + i * 2;

        sclock1 = pct[0].sec * BILLION + pct[0].nsec;
        pclock = pct[1].sec * BILLION + pct[1].nsec;
        sclock2 = pct[2].sec * BILLION + pct[2].nsec;

        window = sclock2 - sclock1;
        if (window < best_window)
        {
            best_window = window;
            best_sclock = sclock1;
            best_pclock = pclock;
        }
    }

    return(best_sclock + best_window / 2L - best_pclock);
}

#else

//
// Delta between two ptp clocks in nanoseconds
//
static long
ptpclock_delta(
    const struct ptp_clock_time * pc1,
    const struct ptp_clock_time * pc2)
{
    struct timespec             ts1;
    struct timespec             ts2;

    ts1.tv_sec = pc1->sec;
    ts1.tv_nsec = pc1->nsec;
    ts2.tv_sec = pc2->sec;
    ts2.tv_nsec = pc2->nsec;

    return(timespec_delta(&ts1, &ts2));
}


//
// Choose the best ptp/sys clock offset by averaging
//
static long
best_ptp_offset(
    struct ptp_sys_offset *     ptp_offset)
{
    long                        window[PTP_MAX_SAMPLES] = {0};
    long long                   total_offset = 0;
    long long                   total_window = 0;
    long                        median_window;
    unsigned int                used = 0;
    struct ptp_clock_time *     pc;
    unsigned int                i;

    // Record the window sizes
    for (i = 0; i < ptp_offset->n_samples; i++)
    {
        pc = ptp_offset->ts + i * 2;
        window[i] = ptpclock_delta(&pc[0], &pc[2]);
    }

    // Use samples that are at or below the median window size
    median_window = median(window, ptp_offset->n_samples);
    for (i = 0; i < ptp_offset->n_samples; i++)
    {
        if (window[i] > median_window)
        {
            continue;
        }

        pc = ptp_offset->ts + i * 2;
        // FIXME warn on overflow
        total_offset += ptpclock_delta(&pc[1], &pc[0]);

        total_window += window[i];
        used++;
    }

    return((long) (total_offset / used + total_window / used / 2L));
}
#endif



//
// Process a control message for timestamp info
//
static void
cmsg_timestamp(
    struct msghdr *             msg,
    struct timespec *           timestamp)
{
    struct cmsghdr *            cm;
    struct timespec *           ts;

    for (cm = CMSG_FIRSTHDR(msg); cm != NULL; cm = CMSG_NXTHDR(msg, cm))
    {
        if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type ==  SO_TIMESTAMPING)
        {
            ts = (struct timespec *) CMSG_DATA(cm);
            if (hwstamps)
            {
                *timestamp = ts[2];
            }
            else
            {
                *timestamp = ts[0];
            }
            return;
        }
    }
}



//
// Receive thread
//
static void *
rx_thread(
    __attribute__ ((unused))
    void *                      arg)
{
    struct iovec                iovec;
    struct pollfd               pfd;
    struct msghdr               msg_hdr;
    char                        cmsg_buf[CMSG_BUF_LEN];
    char                        rx_packet[ETH_FRAME_LEN];
    struct timespec             after_poll;
    struct ptp_sys_offset       ptp_offset;
    long                        len;
    unsigned int                i;
    int                         r;

    // Silence valgrind
    memset(&ptp_offset, 0, sizeof(ptp_offset));

    // Set up for poll
    pfd.fd = rx_socket;
    pfd.events = POLLIN;

    for (i = 0; i < iterations; i++)
    {
        while (1)
        {
            // Set up for rx recvmsg
            iovec.iov_base = &rx_packet;
            iovec.iov_len = sizeof(rx_packet);
            msg_hdr.msg_name = NULL;
            msg_hdr.msg_namelen = 0;
            msg_hdr.msg_iov = &iovec;
            msg_hdr.msg_iovlen = 1;
            msg_hdr.msg_control = &cmsg_buf;
            msg_hdr.msg_controllen = sizeof(cmsg_buf);

            // Wait for the packet to arrive
            r = poll(&pfd, 1, RX_POLL_LIMIT);
            if (r == -1)
            {
                perror("poll");
                fatal("poll for receive socket failed\n");
            }
            else if (r == 0)
            {
                fatal("timed out waiting for rx packet\n");
            }
            clock_gettime(CLOCK_REALTIME, &after_poll);

            if (hwstamps)
            {
                // Record phy vs sys.
                ptp_offset_capture(rx_ptpdev, &ptp_offset);
            }

            len = recvmsg(rx_socket, &msg_hdr, 0);
            if (len == -1)
            {
                perror("recvmsg");
                fatal("receive of packet failed\n");
            }

            // Is this is our packet?
            if (memcmp(tx_packet, rx_packet, sizeof(tx_packet)) == 0)
            {
                // Process the timestamp control message
                if (msg_hdr.msg_controllen)
                {
                    cmsg_timestamp(&msg_hdr, &rx_values[i].timestamp);
                }
                else
                {
                    fatal("no rx control message (timestamp) received\n");
                }

                if (hwstamps)
                {
                    timespec_addns(&rx_values[i].timestamp, best_ptp_offset(&ptp_offset));
                }

                break;
            }
        }

        rx_values[i].timestamp_to_poll_return = timespec_delta(&rx_values[i].timestamp, &after_poll);
    }
    return(0);
}



//
// Send
//
static void
do_send(void)
{
    struct iovec                iovec;
    struct sockaddr_ll          sockaddr_ll;
    struct ethhdr *             hdr;
    struct pollfd               pfd;
    struct msghdr               msg_hdr;
    char                        cmsg_buf[CMSG_BUF_LEN];
    struct timespec             before_send;
    struct timespec             after_send;
    struct timespec             before_poll;
    struct timespec             after_poll;
    struct ptp_sys_offset       ptp_offset;
    char                        tx_packet_copy[ETHER_PACKET_BYTES];
    long                        len;
    unsigned int                i;
    int                         r;

    // Safety checks
    assert(sizeof(hdr) <= sizeof(tx_packet));
    assert(sizeof(rx_addr) <= sizeof(hdr->h_dest));
    assert(sizeof(tx_addr) <= sizeof(hdr->h_source));
    assert(sizeof(rx_addr) <= sizeof(sockaddr_ll.sll_addr));

    // Silence valgrind
    memset(&ptp_offset, 0, sizeof(ptp_offset));

    // Format ethernet header
    hdr = (struct ethhdr *) &tx_packet;
    memcpy(&hdr->h_dest, &rx_addr, sizeof(rx_addr));
    memcpy(&hdr->h_source, &tx_addr, sizeof(tx_addr));
    hdr->h_proto = htons(ETHER_PROTO);

    // Destination address
    memset(&sockaddr_ll, 0, sizeof(sockaddr_ll));
    sockaddr_ll.sll_family = AF_PACKET;
    sockaddr_ll.sll_ifindex = tx_index;
    sockaddr_ll.sll_halen = sizeof(rx_addr);
    memcpy(&sockaddr_ll.sll_addr, &rx_addr, sizeof(rx_addr));

    // Set up for poll
    pfd.fd = tx_socket;
    pfd.events = POLLPRI;

    for (i = 0; i < iterations; i++)
    {
        // Set up for recvmsg
        iovec.iov_base = &tx_packet_copy;
        iovec.iov_len = sizeof(tx_packet_copy);
        msg_hdr.msg_name = NULL;
        msg_hdr.msg_namelen = 0;
        msg_hdr.msg_name = NULL;
        msg_hdr.msg_iov = &iovec;
        msg_hdr.msg_iovlen = 1;
        msg_hdr.msg_control = &cmsg_buf;
        msg_hdr.msg_controllen = sizeof (cmsg_buf);

        delay(TX_SEND_DELAY);
        if (hwstamps)
        {
            // Record phy vs sys
            ptp_offset_capture(tx_ptpdev, &ptp_offset);
        }

        clock_gettime(CLOCK_REALTIME, &before_send);
        // Valgrind reports that sendto reads beyond the end of the sockaddr_ll. There doesn't seem
        // to be a reasonable way to prevent this.
        len = sendto(tx_socket, tx_packet, sizeof(tx_packet), 0, (struct sockaddr *) &sockaddr_ll, sizeof(sockaddr_ll));
        if (len == -1 || len != sizeof(tx_packet))
        {
            perror("sendto");
            fatal("send of packet from interface %s failed\n", tx_name);
        }
        clock_gettime(CLOCK_REALTIME, &after_send);

        // Wait for the timestamp
        clock_gettime(CLOCK_REALTIME, &before_poll);
        r = poll(&pfd, 1, TX_POLL_LIMIT);
        if (r == -1)
        {
            perror("poll");
            fatal("poll for tx timestamp failed\n");
        }
        else if (r == 0)
        {
            fatal("timed out waiting for tx timestamp\n");
        }
        clock_gettime(CLOCK_REALTIME, &after_poll);

        len = recvmsg(tx_socket, &msg_hdr, MSG_ERRQUEUE);
        if (len == -1)
        {
            perror("recvmsg");
            fatal("receive of tx timestamp failed\n");
        }

        // Safety check
        if (memcmp(tx_packet, tx_packet_copy, sizeof(tx_packet)) != 0)
        {
            fatal("send and timestamp packet do not match\n");
        }

        // Process the timestamp control message
        if (msg_hdr.msg_controllen)
        {
            cmsg_timestamp(&msg_hdr, &tx_values[i].timestamp);
        }
        else
        {
            fatal("no tx control message (timestamp) received\n");
        }
        if (hwstamps)
        {
            timespec_addns(&tx_values[i].timestamp, best_ptp_offset(&ptp_offset));
        }

        tx_values[i].before_send_to_after_send = timespec_delta(&before_send, &after_send);
        tx_values[i].after_send_to_before_poll = timespec_delta(&after_send, &before_poll);
        tx_values[i].after_send_to_after_poll = timespec_delta(&after_send, &after_poll);
        tx_values[i].before_send_to_timestamp = timespec_delta(&before_send, &tx_values[i].timestamp);
    }
}



//
// Output usage
//
static void
usage(void)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s [-S] [-i iterations] [-r rx_comp] [-t tx_comp] [-s sw_comp] [-c cable_len] [-p ptp_samples] [-d file] src_iface dst_iface\n\n", progname);
    fprintf(stderr, "  options:\n");
    fprintf(stderr, "    -S use software timestamps instead of hardware timestamps (low accuracy)\n");
    fprintf(stderr, "    -i number of iterations (default 1)\n");
    fprintf(stderr, "    -r receive compensation in nanoseconds\n");
    fprintf(stderr, "    -t receive compensation in nanoseconds\n");
    fprintf(stderr, "    -s switch compensation in nanoseconds\n");
    fprintf(stderr, "    -c cable length in meters (default 0.0)\n");
    fprintf(stderr, "    -p number of ptp clock samples to request\n");
    fprintf(stderr, "    -d dump raw data to file\n");
}



//
// Parse command line arguments
//
static void
parse_args(
    int                         argc,
    char * const                argv[])
{
    int                         opt;
    long                        l;

    progname = argv[0];

    while((opt = getopt(argc, argv, "Si:r:t:s:c:p:d:")) != -1)
    {
        switch (opt)
        {
        case 'S':
            hwstamps = 0;
            break;
        case 'i':
            l = strtol(optarg, NULL, 10);
            if (l < 1)
            {
                fatal("iterations must be at least 1\n");
            }
            iterations = (unsigned int) l;
            break;
        case 'r':
            rx_comp = strtol(optarg, NULL, 10);
            break;
        case 't':
            tx_comp = strtol(optarg, NULL, 10);
            break;
        case 's':
            sw_comp = strtol(optarg, NULL, 10);
            break;
        case 'c':
            cable_len = strtod(optarg, NULL);
            break;
        case 'p':
            ptp_samples = (unsigned int) strtoul(optarg, NULL, 10);
            if (ptp_samples > PTP_MAX_SAMPLES)
            {
                fatal("ptp_samples must be less than or equal to %d\n", PTP_MAX_SAMPLES);
            }
            break;
        case 'd':
            data_file_name = optarg;
            break;

        default:
            usage();
            exit(EXIT_FAILURE);
        }
    }

    // Ensure we have the correct number of parameters
    if (argc != optind + 2)
    {
        usage();
        exit(EXIT_FAILURE);
    }

    tx_name = argv[optind++];
    rx_name = argv[optind++];
}



//
// Main
//
int
main(
    int                         argc,
    char                        *argv[])
{
    pthread_t                   thread_id;
    long                        sfsw_expected;
    long                        ctsw_expected;
    long                        cable_comp;

    struct ptp_sys_offset       ptp_offset;
    long                        tx_initial_offset = 0;
    long                        rx_initial_offset = 0;
    long                        tx_final_offset = 0;
    long                        rx_final_offset = 0;

    long long                   tx_before_send_to_after_send_total = 0;
    long long                   tx_before_send_to_timestamp_total = 0;
    long long                   tx_after_send_to_before_poll_total = 0;
    long long                   tx_after_send_to_after_poll_total = 0;
    long long                   rx_timestamp_to_poll_return_total = 0;
    long long                   tx_rx_timestamp_total = 0;
    unsigned long long          tx_rx_timestamp_total2 = 0;
    long *                      tx_rx_deltas;

    long                        tx_before_send_to_after_send;
    long                        tx_before_send_to_timestamp;
    long                        tx_after_send_to_before_poll;
    long                        tx_after_send_to_after_poll;
    long                        rx_timestamp_to_poll_return;

    long                        delta;
    long                        tx_rx_raw;
    unsigned long               tx_rx_raw_stddev;
    long                        tx_rx_raw_median;
    long                        tx_rx_compensated;

    unsigned int                i;
    int                         r;

    // Handle command line args
    parse_args(argc, argv);

    if (data_file_name)
    {
        data_file = fopen(data_file_name, "w");
        if (data_file == NULL)
        {
            perror("fopen");
            fatal("cannot open data file\n");
        }
    }

    tx_values = calloc((size_t) iterations, sizeof(*tx_values));
    rx_values = calloc((size_t) iterations, sizeof(*rx_values));
    tx_rx_deltas = calloc((size_t) iterations, sizeof(*tx_rx_deltas));
    assert(tx_values);
    assert(rx_values);

    // Create the sockets
    tx_socket = socket_create(tx_name, &tx_index, &tx_ptp_index, &tx_addr, &tx_speed);
    rx_socket = socket_create(rx_name, &rx_index, &rx_ptp_index, &rx_addr, &rx_speed);

    if (tx_index == rx_index)
    {
        printf("WARNING: Transmit and receive interfaces are the same. Results are unpredictable\n");
        fflush(stdout);
        delay(WARNING_DELAY);
    }

    if (hwstamps)
    {
        // Open the ptp descriptors
        tx_ptpdev = ptpdev_open(tx_ptp_index);
        rx_ptpdev = ptpdev_open(rx_ptp_index);
    }
    else
    {
        printf("WARNING: Using software timestamps instead of hardware. Results will be inaccurate.\n\n");
    }

    // Drain any pending messages and clear any error state.
    //
    //   NB: It's unclear if explictly draining the error queue is actually necessary.
    //
    drain(tx_socket, MSG_ERRQUEUE);
    drain(tx_socket, 0);
    drain(rx_socket, MSG_ERRQUEUE);
    drain(rx_socket, 0);

    // Create receive thread
    r = pthread_create(&thread_id, NULL, &rx_thread, NULL);
    if (r != 0)
    {
        perror("pthread_create");
        fatal("cannot create receive thread\n");
    }

    cable_comp = (long) (cable_len * NSEC_PER_METER);
    sfsw_expected = ETHER_SFSW_TX_BITS * 1000L / (long) tx_speed +
                    ETHER_SFSW_RX_BITS * 1000L / (long) rx_speed +
                    sw_comp + cable_comp;
    ctsw_expected = ETHER_CTSW_TX_BITS * 1000L / (long) tx_speed +
                    ETHER_CTSW_RX_BITS * 1000L / (long) rx_speed +
                    sw_comp + cable_comp;

    if (iterations > 1000)
    {
        printf("Iterating over %u packets (minimum runtime %ld seconds)\n\n", iterations,
               (START_DELAY + (TX_SEND_DELAY) * (long) iterations) / MILLION);
        fflush(stdout);
    }

    // Ensure receive thread is ready
    delay(START_DELAY);

    if (hwstamps)
    {
        ptp_offset_capture(tx_ptpdev, &ptp_offset);
        tx_initial_offset = best_ptp_offset(&ptp_offset);
        ptp_offset_capture(rx_ptpdev, &ptp_offset);
        rx_initial_offset = best_ptp_offset(&ptp_offset);
    }

    // Do send
    do_send();

    // Wait for receive thread to return
    pthread_join(thread_id, NULL);

    if (hwstamps)
    {
        ptp_offset_capture(tx_ptpdev, &ptp_offset);
        tx_final_offset = best_ptp_offset(&ptp_offset);
        ptp_offset_capture(rx_ptpdev, &ptp_offset);
        rx_final_offset = best_ptp_offset(&ptp_offset);
    }

    for (i = 0; i < iterations; i++)
    {
        tx_before_send_to_after_send_total += tx_values[i].before_send_to_after_send;
        tx_before_send_to_timestamp_total += tx_values[i].before_send_to_timestamp;
        tx_after_send_to_before_poll_total += tx_values[i].after_send_to_before_poll;
        tx_after_send_to_after_poll_total += tx_values[i].after_send_to_after_poll;
        rx_timestamp_to_poll_return_total += rx_values[i].timestamp_to_poll_return;

        delta = timespec_delta(&tx_values[i].timestamp, &rx_values[i].timestamp);
#if 0
        if (delta < 0)
        {
            printf("Time warp in delta\n");
        }
#endif

        tx_rx_deltas[i] = delta;
        tx_rx_timestamp_total += delta;
        tx_rx_timestamp_total2 += (unsigned long long) (delta * delta);

        if (data_file)
        {
            fprintf(data_file, "%ld\n", delta);
        }
    }

    tx_before_send_to_after_send = tx_before_send_to_after_send_total / (long) iterations;
    tx_before_send_to_timestamp = tx_before_send_to_timestamp_total / (long) iterations;
    tx_after_send_to_before_poll = tx_after_send_to_before_poll_total / (long) iterations;
    tx_after_send_to_after_poll = tx_after_send_to_after_poll_total / (long) iterations;
    rx_timestamp_to_poll_return = rx_timestamp_to_poll_return_total / (long) iterations;

    tx_rx_raw =  tx_rx_timestamp_total / (long) iterations;
    tx_rx_raw_stddev = llsqrt((tx_rx_timestamp_total2 / iterations) - ((unsigned long long) (tx_rx_raw * tx_rx_raw)));
    tx_rx_raw_median = median(tx_rx_deltas, iterations);
    tx_rx_compensated = tx_rx_raw - tx_comp - rx_comp;

    printf("Tx interface %s, speed %dMb, adddress %s\n", tx_name, tx_speed, ether_ntoa(&tx_addr));
    printf("Rx interface %s, speed %dMb, adddress %s\n", rx_name, rx_speed, ether_ntoa(&rx_addr));
    if (hwstamps)
    {
        printf("Tx drift during run %ld\n", tx_initial_offset - tx_final_offset);
        printf("Rx drift during run %ld\n", rx_initial_offset - rx_final_offset);
    }
    printf("\n");

    printf("Misc times:\n");
    printf("  Tx before send  ->  after send   %8ldns\n", tx_before_send_to_after_send);
    printf("  Tx before send  ->  tx timestamp %8ldns\n", tx_before_send_to_timestamp);
    printf("  Tx after send   ->  before poll  %8ldns\n", tx_after_send_to_before_poll);
    printf("  Tx after send   ->  after poll   %8ldns\n", tx_after_send_to_after_poll);
    printf("  Rx timestamp    ->  poll return  %8ldns\n", rx_timestamp_to_poll_return);
    printf("\n");
    printf("Compensation values:\n");
    printf("  Tx timestamp                     %8ldns\n", tx_comp);
    printf("  Rx timestamp                     %8ldns\n", rx_comp);
    printf("  Switch port to port              %8luns\n", sw_comp);
    printf("  Cable length                     %8luns  (%.1fm)\n", cable_comp, cable_len);
    printf("  Total                            %8ldns\n\n", tx_comp + rx_comp + sw_comp + cable_comp);

    printf("Tx -> Rx timestamp:\n");
    printf("  Median                           %8ldns\n", tx_rx_raw_median);
    printf("  Average                          %8ldns  (stddev %ldns)\n", tx_rx_raw, tx_rx_raw_stddev);
    printf("  Compensated                      %8ldns\n\n", tx_rx_compensated);

    printf("Connection types:       Expected        Error\n");
    if (tx_speed == rx_speed)
    {
        printf("  loopback cable      %8luns   %8ldns\n", cable_comp, tx_rx_compensated - cable_comp);
    }
    if (tx_speed >= rx_speed)
    {
        printf("  Cut-through switch  %8luns   %8ldns\n", ctsw_expected, tx_rx_compensated - ctsw_expected);
    }
    printf("  Regular switch      %8luns   %8ldns\n", sfsw_expected, tx_rx_compensated - sfsw_expected);

    exit(0);
}
