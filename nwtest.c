/*
 *   BSD LICENSE
 * 
 *   Copyright(c) 2014 Hiroshi Shimamoto All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_pci.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_cycles.h>
#include <rte_byteorder.h>
#include <rte_ether.h>

static uint64_t freq;

static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.jumbo_frame = 0,
	},
	.txmode = {
		.mq_mode = ETH_MQ_TX_NONE,
	},
};

static const struct rte_eth_rxconf rx_conf = {
	.rx_thresh = {
		.pthresh = 8,
		.hthresh = 8,
		.wthresh = 4,
	},
};

static const struct rte_eth_txconf tx_conf = {
	.tx_thresh = {
		.pthresh = 36,
		.hthresh = 0,
		.wthresh = 0,
	},
};

static struct rte_mempool *nwtest_pool;
static const unsigned MBUF_SIZE = (2048 + sizeof(struct rte_mbuf) +
					RTE_PKTMBUF_HEADROOM);
static const unsigned NR_MBUF = 8192;
static const unsigned MBUF_CACHE_SIZE = 32;

static const unsigned nr_rxdesc = 128;
static const unsigned nr_txdesc = 512;

static unsigned sender_lcore = 0;
static unsigned receiver_lcore = 1;

static int parse_args(int argc, char **argv)
{
	return 0;
}

static void init_port(unsigned portid)
{
	if (rte_eth_dev_configure(portid, 1, 1, &port_conf) < 0) {
		rte_exit(EXIT_FAILURE, "failed to configure port %u\n",
			portid);
	}
	if (rte_eth_rx_queue_setup(portid, 0, nr_rxdesc,
			rte_eth_dev_socket_id(portid), &rx_conf,
			nwtest_pool) < 0) {
		rte_exit(EXIT_FAILURE,
			"failed to configure rx queue port %u\n",
			portid);
	}
	if (rte_eth_tx_queue_setup(portid, 0, nr_txdesc,
			rte_eth_dev_socket_id(portid), &tx_conf) < 0) {
		rte_exit(EXIT_FAILURE,
			"failed to configure tx queue port %u\n",
			portid);
	}

	rte_eth_promiscuous_enable(portid);

	if (rte_eth_dev_start(portid) < 0) {
		rte_exit(EXIT_FAILURE, "failed to start device port %u\n",
			portid);
	}
}

static int setup_env(void)
{
	unsigned nr_ports;
	unsigned portid;
	uint64_t tsc;

	tsc = rte_rdtsc();
	sleep(1);
	freq = rte_rdtsc() - tsc;

	if (rte_pmd_init_all() < 0)
		rte_exit(EXIT_FAILURE, "failed to init pmd\n");

	if (rte_eal_pci_probe() < 0)
		rte_exit(EXIT_FAILURE, "failed to probe PCI\n");

	nwtest_pool = rte_mempool_create("nwtest_pool", NR_MBUF,
				MBUF_SIZE, MBUF_CACHE_SIZE,
				sizeof(struct rte_pktmbuf_pool_private),
				rte_pktmbuf_pool_init, NULL,
				rte_pktmbuf_init, NULL,
				rte_socket_id(), 0);
	if (!nwtest_pool)
		rte_exit(EXIT_FAILURE, "failed to create mempool\n");

	nr_ports = rte_eth_dev_count();
	for (portid = 0; portid < nr_ports; portid++)
		init_port(portid);

	return 0;
}

/*
 * UDP packet layout
 * | Ether header | +14
 * | IP header    | +20
 * | UDP header   | + 8
 * | UDP data     | >18
 * | FCS          | + 4
 * | TOTAL        | >64
 */
struct udp_data {
	uint16_t seq;
	uint64_t send_tsc;
	uint64_t recv_tsc;
} __attribute__((__packed__));

static unsigned frame_len = 64;

static uint8_t frame[4096];

static struct ether_addr dstmac = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
static struct ether_addr srcmac = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };

/* rate */
static uint64_t burst_pkts, burst_nsec;

/* statistics */
static uint64_t pkts_out;
static uint64_t pkts_in;
static uint64_t hist[10], over;
static uint64_t units[10];

static void setup_frame(void)
{
	struct ether_hdr *ehdr;

	ehdr = (struct ether_hdr *)&frame[0];
	rte_memcpy(&ehdr->d_addr, &dstmac, sizeof(dstmac));
	rte_memcpy(&ehdr->s_addr, &srcmac, sizeof(srcmac));
	ehdr->ether_type = rte_cpu_to_be_16(ETHER_TYPE_IPv4);
}

static int generate(struct rte_mbuf **bufs, uint16_t seq, uint64_t tsc, int nr)
{
	struct rte_mbuf *m;
	int i;
	unsigned len = frame_len - 4;
	struct udp_data *u;

	u = (struct udp_data *)&frame[42];
	u->seq = seq;
	u->send_tsc = tsc;

	for (i = 0; i < nr; i++) {
		struct rte_mbuf *m;

		m = rte_pktmbuf_alloc(nwtest_pool);
		if (!m)
			break;
		m->pkt.data_len = len;
		m->pkt.pkt_len = len;
		m->pkt.nb_segs = 1;
		m->pkt.next = NULL;

		rte_memcpy(m->pkt.data, frame, len);

		bufs[i] = m;
	}

	return i;
}

static void sender_thread(void)
{
	uint16_t seq = 0;
	unsigned portid = 0;
	uint64_t blank;

	if (burst_nsec)
		blank = freq / (1000000000 / burst_nsec);
	else
		blank = 0;

	burst_pkts = 10;

	for (;;) {
		int nr, burst = burst_pkts;
		int nr_tx, i;
		uint64_t tsc = rte_rdtsc();
		uint64_t wait = tsc + blank;

		while (burst > 0) {
			struct rte_mbuf *bufs[100];

			nr = generate((struct rte_mbuf **)&bufs, seq, tsc, burst);

			nr_tx = rte_eth_tx_burst(portid, 0, bufs, nr);
			for (i = nr_tx; i < nr; i++)
				rte_pktmbuf_free(bufs[i]);
			pkts_out += nr_tx;
			burst -= nr_tx;
		}

		while (rte_rdtsc() < wait)
			rte_pause();

		++seq;
	}
}

static void sink(struct rte_mbuf **bufs, int nr)
{
	uint64_t tsc = rte_rdtsc();
	int i;

	for (i = 0; i < nr; i++) {
		void *data = rte_pktmbuf_mtod(bufs[i], void *);
		struct udp_data *u = data + 42;
		uint64_t tat;
		int h;

		tat = tsc - u->send_tsc;
		for (h = 0; h < 10; h++) {
			if (tat < units[h]) {
				hist[h]++;
				goto next;
			}
		}
		over++;

next:
		rte_pktmbuf_free(bufs[i]);
	}
}

static void receiver_thread(void)
{
	unsigned portid = 0;

	for (;;) {
		struct rte_mbuf *bufs[10];
		int nr_rx, i;

		nr_rx = rte_eth_rx_burst(portid, 0, bufs, 10);
		if (nr_rx == 0)
			continue;

		sink(bufs, nr_rx);

		pkts_in += nr_rx;
	}
}

static int nwtest(void *p)
{
	unsigned lcore_id = rte_lcore_id();

	if (lcore_id == sender_lcore)
		sender_thread();
	else if (lcore_id == receiver_lcore)
		receiver_thread();

	return 0;
}

int main(int argc, char **argv)
{
	int ret;
	unsigned lcore_id;
	int i;
	uint64_t unit;

	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
	argc -= ret;
	argv += ret;

	if (parse_args(argc, argv))
		rte_exit(EXIT_FAILURE, "Invalid arguments\n");

	setup_env();
	setup_frame();

	/* micro sec */
	unit = freq / 1000000;
	for (i = 0; i < 10; i++) {
		units[i] = unit;
		unit *= 10;
	}

	rte_eal_mp_remote_launch(nwtest, NULL, CALL_MASTER);
	RTE_LCORE_FOREACH_SLAVE(lcore_id) {
		if (rte_eal_wait_lcore(lcore_id) < 0)
			return -1;
	}

	return 0;
}
