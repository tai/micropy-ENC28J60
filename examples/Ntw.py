#!/usr/bin/env python
# -*- coding: utf8 -*-

# Copyright 2021 Przemyslaw Bereski https://github.com/przemobe/

# This is version for MicroPython v1.17

# This file implements very simple IP stack for ENV28J60 ethernet.
# Supports:
# - ARP for IPv4 over Ethernet, simple ARP table
# - IPv4 for not fragmented packets only, single static IP address
# - ICMPv4: rx Echo Request and tx Echo Response
# - UDPv4: rx and tx


from machine import Pin
from machine import SPI
from micropython import const
from enc28j60 import enc28j60
import struct


ETH_TYPE_IP4   = const(0x0800)
ETH_TYPE_ARP   = const(0x0806)
ETH_80211Q_TAG = const(0x8100)

ARP_HEADER_LEN = const(28)
ARP_OP_REQUEST = const(1)
ARP_OP_REPLY   = const(2)

IP4_TYPE_ICMP = const(1)
IP4_TYPE_TCP  = const(6)
IP4_TYPE_UDP  = const(17)
IP4_ADDR_BCAST  = bytearray([255,255,255,255])
IP4_ADDR_ZERO   = bytearray([0,0,0,0])

ICMP4_ECHO_REPLY    = const(0)
ICMP4_UNREACHABLE   = const(3)
ICMP4_ECHO_REQUEST  = const(8)


class Packet:
    '''This class stores received packet information'''
    def __init__(self, ntw, frame):
        self.ntw = ntw
        self.frame = memoryview(frame)


def procArp(pkt):
    offset = pkt.eth_offset
    hrtype, prtype, hrlen, prlen, oper = struct.unpack("!HHBBH", pkt.frame[0+offset:8+offset])
    sha = pkt.frame[8+offset:14+offset]
    spa = pkt.frame[14+offset:18+offset]
    tha = pkt.frame[18+offset:24+offset]
    tpa = pkt.frame[24+offset:28+offset]

    print(f'Rx ARP oper={oper}')

    if ARP_OP_REQUEST == oper:
        if tpa == pkt.ntw.myIp4Addr:
            print(f'Rx ARP_REQUEST for my IP from IP {spa[0]}.{spa[1]}.{spa[2]}.{spa[3]}!')
            reply = makeArpReply(pkt.eth_src, pkt.ntw.myMacAddr, pkt.ntw.myIp4Addr, spa)
            n = pkt.ntw.txPkt(reply)
            if 0 > n:
                print(f'Fail to send ARP REPLY {n}')
    elif ARP_OP_REPLY == oper:
        print(f'ARP {spa[0]}.{spa[1]}.{spa[2]}.{spa[3]} is at {sha[0]:02X}:{sha[1]:02X}:{sha[2]:02X}:{sha[3]:02X}:{sha[4]:02X}:{sha[5]:02X}')
        pkt.ntw.addArpEntry(spa, sha)


def makeArpReply(eth_dst, eth_src, ip_src, ip_dst):
    rsp = []
    rsp.append(eth_dst)
    rsp.append(eth_src)
    rsp.append(bytearray([ETH_TYPE_ARP >> 8, ETH_TYPE_ARP, 0, 1, 8, 0, 6, 4, 0, ARP_OP_REPLY]))
    rsp.append(eth_src)
    rsp.append(ip_src)
    rsp.append(eth_dst)
    rsp.append(ip_dst)
    return rsp


def makeArpRequest(eth_src, ip_src, ip_dst):
    rsp = []
    rsp.append(bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]))
    rsp.append(eth_src)
    rsp.append(bytearray([ETH_TYPE_ARP >> 8, ETH_TYPE_ARP, 0, 1, 8, 0, 6, 4, 0, ARP_OP_REQUEST]))
    rsp.append(eth_src)
    rsp.append(ip_src)
    rsp.append(bytearray(6))
    rsp.append(ip_dst)
    return rsp


def calcChecksum(data, startValue = 0):
    chksm = startValue
    for idx in range(0, len(data)-1, 2):
        chksm += (data[idx] << 8) | data[idx+1]
    if len(data) & 0x1:
        chksm += data[-1] << 8
    chksm = (chksm >> 16) + (chksm & 0xffff)
    chksm += (chksm >> 16)
    return ~chksm & 0xffff


def makeIp4Hdr(src, tgt, ident, prot, dataLen, ttl=128, dscp=0, ecn=0):
    totlen = 20 + dataLen
    hdr = bytearray(20)
    hdr[0] = 0x45   # Version + IHL
    hdr[1] = (dscp << 2) | (ecn & 0x03)
    hdr[2] = totlen >> 8
    hdr[3] = totlen
    hdr[4] = ident >> 8
    hdr[5] = ident
    hdr[6] = 0      # Flags + Fragment Offset
    hdr[7] = 0      # Flags + Fragment Offset
    hdr[8] = ttl
    hdr[9] = prot
    hdr[10] = 0
    hdr[11] = 0
    hdr[12:16] = src
    hdr[16:20] = tgt

    chksm = calcChecksum(hdr)
    hdr[10] = (chksm >> 8) & 0xFF
    hdr[11] = chksm & 0xFF
    return hdr


def sendIcmp4EchoReply(pkt):
    offset = pkt.ip_offset
    rsp = []

    # ICMP
    icmpRepl = bytearray(pkt.frame[offset:pkt.ip_maxoffset])
    icmpRepl[0] = ICMP4_ECHO_REPLY
    icmpRepl[1] = 0x00
    icmpRepl[2] = 0x00
    icmpRepl[3] = 0x00
    chksm = calcChecksum(icmpRepl)
    icmpRepl[2] = (chksm >> 8) & 0xFF
    icmpRepl[3] = chksm & 0xFF

    # IP
    ipHdr = makeIp4Hdr(pkt.ntw.myIp4Addr, pkt.ip_src_addr, pkt.ntw.ip4TxCount, IP4_TYPE_ICMP, len(icmpRepl))
    pkt.ntw.ip4TxCount += 1

    # Eth
    rsp.append(pkt.eth_src)
    rsp.append(pkt.ntw.myMacAddr)
    rsp.append(bytearray([ETH_TYPE_IP4 >> 8, ETH_TYPE_IP4]))

    rsp.append(ipHdr)
    rsp.append(icmpRepl)

    n = pkt.ntw.txPkt(rsp)
    return n


def procIcmp4(pkt):
    offset = pkt.ip_offset
    if ICMP4_ECHO_REQUEST == pkt.frame[offset]:
        sendIcmp4EchoReply(pkt)
    else:
        print(f'Rx ICMP op={pkt.frame[offset]}')


def procIp4(pkt):
    offset = pkt.eth_offset
    ip_hdr = struct.unpack("!BBHHHBBH", pkt.frame[0+offset:12+offset])
    pkt.ip_src_addr = pkt.frame[12+offset:16+offset]
    pkt.ip_dst_addr = pkt.frame[16+offset:20+offset]
    pkt.ip_ver = (ip_hdr[0] >> 4) & 0xF
    pkt.ip_hdrlen = (ip_hdr[0] & 0xF) << 2
    pkt.ip_totlen = ip_hdr[2]
    pkt.ip_proto = ip_hdr[6]
    pkt.ip_offset = pkt.eth_offset + pkt.ip_hdrlen
    pkt.ip_maxoffset = pkt.eth_offset + pkt.ip_totlen
    pkt.ntw.ip4RxCount += 1

    #print('ip_hdr', ip_hdr, pkt.ip_hdrlen, pkt.ip_dst_addr[0], pkt.ip_dst_addr[1], pkt.ip_dst_addr[2], pkt.ip_dst_addr[3])

    if 4 != pkt.ip_ver:
        print(f'ip_ver={pkt.ip_ver} not supported!')
        return

    if 20 != pkt.ip_hdrlen:
        print(f'ip_hdrlen={pkt.ip_hdrlen} not supported!')
        return

    #chksm = calcChecksum(pkt.frame[offset:offset+pkt.ip_hdrlen])
    #if 0 != chksm:
        #print(f'IPv4 chksm={chksm} invalid!')
        #return

    flags_mf = (ip_hdr[4] >> 13) & 0x01
    fragOffset = (ip_hdr[4] & 0x1FFF) << 3
    if (0 != flags_mf) or (0 != fragOffset):
        print(f'Fragmented IPv4 not supported: fragOffset={fragOffset}, flags_mf={flags_mf}')
        return

    if pkt.ip_dst_addr == pkt.ntw.myIp4Addr:
        print(f'Rx my IP proto={pkt.ip_proto}')
        if IP4_TYPE_ICMP == pkt.ip_proto:
            procIcmp4(pkt)
        elif IP4_TYPE_TCP == pkt.ip_proto:
            pass
        elif IP4_TYPE_UDP == pkt.ip_proto:
            procUdp4(pkt, bcast=False)
    elif pkt.ip_dst_addr == IP4_ADDR_BCAST:
        if IP4_TYPE_UDP == pkt.ip_proto:
            procUdp4(pkt, bcast=True)


def printEthPkt(pkt):
    print('DST:', ":".join("{:02x}".format(c) for c in pkt[0:6]),
          'SRC:', ":".join("{:02x}".format(c) for c in pkt[6:12]),
          'Type:', ":".join("{:02x}".format(c) for c in pkt[12:14]),
          'len:', len(pkt),
          'FCS', ":".join("{:02x}".format(c) for c in pkt[-4:]))


def procEth(pkt):
    #printEthPkt(pkt.frame)

    pkt.eth_dst = pkt.frame[0:6]
    pkt.eth_src = pkt.frame[6:12]
    pkt.eth_type = struct.unpack("!H", pkt.frame[12:14])[0]
    pkt.eth_offset = 14

    if ETH_80211Q_TAG == pkt.eth_type:
        pkt.eth_type = struct.unpack("!H", pkt.frame[14:16])[0]
        pkt.eth_offset = 16

    if ETH_TYPE_IP4 == pkt.eth_type:
        procIp4(pkt)
    elif ETH_TYPE_ARP == pkt.eth_type:
        procArp(pkt)
    # ignore not supported types


def makeUdp4Hdr(srcIp, srcPort, dstIp, dstPort, data):
    udpHdr = bytearray(8)
    udpLen = len(data) + 8

    chksm = sum(struct.unpack('!HH', srcIp))
    chksm += sum(struct.unpack('!HH', dstIp))
    chksm += IP4_TYPE_UDP + 2*udpLen + srcPort + dstPort
    chksm = calcChecksum(data, chksm)

    udpHdr = bytearray(8)
    udpHdr[0] = srcPort >> 8
    udpHdr[1] = srcPort
    udpHdr[2] = dstPort >> 8
    udpHdr[3] = dstPort
    udpHdr[4] = udpLen >> 8
    udpHdr[5] = udpLen
    udpHdr[6] = chksm >> 8
    udpHdr[7] = chksm
    return udpHdr


def procUdp4(pkt, bcast=False):
    offset = pkt.ip_offset
    pkt.udp_srcPort, pkt.udp_dstPort, udpLen, chksm_rx = struct.unpack('!HHHH', pkt.frame[offset:offset+8])
    pkt.udp_dataLen = udpLen - 8
    pkt.udp_data = memoryview(pkt.frame[offset+8:offset+udpLen])

    # find UDP client
    cb = None
    if (False == bcast) and (pkt.udp_dstPort in pkt.ntw.udp4UniBind):
        cb = pkt.ntw.udp4UniBind[pkt.udp_dstPort]
    elif (True == bcast) and (pkt.udp_dstPort in pkt.ntw.udp4BcastBind):
        cb = pkt.ntw.udp4BcastBind[pkt.udp_dstPort]

    if None == cb:
        return

    # verify checksum
    if (0 != chksm_rx):
        chksm = sum(struct.unpack('!HH', pkt.ip_src_addr))
        chksm += sum(struct.unpack('!HH', pkt.ip_dst_addr))
        chksm += IP4_TYPE_UDP + 2*udpLen + pkt.udp_srcPort + pkt.udp_dstPort
        chksm = calcChecksum(pkt.udp_data, chksm)
        if 0 == chksm:
            chksm = 0xFFFF
        if (chksm != chksm_rx):
            print(f'Invalid UDP chksm: rx={chksm_rx:04X} calc=0x{chksm:04X}')
            return

    # call UDP client
    cb(pkt)


class Ntw:
    def __init__(self):
        self.rxBuff = bytearray(enc28j60.ENC28J60_ETH_RX_BUFFER_SIZE)
        self.spi1 = SPI(1, baudrate=10000000, sck=Pin(10), mosi=Pin(11), miso=Pin(8))
        self.nic = enc28j60.ENC28J60(self.spi1, Pin(13))

        # Eth settings
        self.myMacAddr = self.nic.getMacAddr()

        # IPv4 settings
        self.myIp4Addr = bytearray(4)
        self.netIp4Mask = bytearray(4)
        self.gwIp4Addr = bytearray(4)

        # Stats
        self.ip4TxCount = 0
        self.ip4RxCount = 0

        self.arpTable = {}
        self.udp4UniBind = {}   # {port:callback(Pkt)}
        self.udp4BcastBind = {} # {port:callback(Pkt)}

        self.nic.init()

        print("MAC ADDR:", ":".join("{:02x}".format(c) for c in self.myMacAddr))
        print("ENC28J60 revision ID: 0x{:02x}".format(self.nic.GetRevId()))

    def setIPv4(self, myIp4Addr, netIp4Mask, gwIp4Addr):
        self.myIp4Addr = bytearray(myIp4Addr)
        self.netIp4Mask = bytearray(netIp4Mask)
        self.gwIp4Addr = bytearray(gwIp4Addr)

    def rxAllPkt(self):
        '''Function to rx and process all pending packets from NIC'''
        while True:
            ## lock
            rxPacketCnt = self.nic.GetRxPacketCnt()
            if 0 == rxPacketCnt:
                ## unlock
                break
            rxLen = self.nic.ReceivePacket(self.rxBuff)
            ## unlock
            if 0 >= rxLen:
                print(f'Rx ERROR {rxLen}')
                continue
            procEth(Packet(self, self.rxBuff))

    def txPkt(self, msg):
        '''Function to tx packet to NIC'''
        ## lock
        n = self.nic.SendPacket(msg)
        ## unlock
        return n

    def registerUdp4Callback(self, port, cb):
        if None != cb:
            self.udp4UniBind[port] = cb
        else:
            self.udp4UniBind.pop(port, None)

    def registerUdp4BcastCallback(self, port, cb):
        if None != cb:
            self.udp4BcastBind[port] = cb
        else:
            self.udp4BcastBind.pop(port, None)

    def addArpEntry(self, ip, mac):
        if type(ip) == int:
            self.arpTable[ip] = bytearray(mac)
        else:
            self.arpTable[struct.unpack('!I',ip)[0]] = bytearray(mac)

    def getArpEntry(self, ip):
        if type(ip) != int:
            ip = struct.unpack('!I',ip)[0]

        if ip in self.arpTable:
            return self.arpTable[ip]
        else:
            return None

    def sendArpRequest(self, ip4Addr):
        msg = makeArpRequest(self.myMacAddr, self.myIp4Addr, ip4Addr)
        n = self.txPkt(msg)
        return n

    def sendUdp4(self, tgt_ip, tgt_port, data, src_port=0):
        msg = []

        tgtMac = self.getArpEntry(tgt_ip)
        if None == tgtMac:
            print('IP address not in ARP table!')
            return -1

        msg.append(tgtMac)
        msg.append(self.myMacAddr)
        msg.append(bytearray([ETH_TYPE_IP4 >> 8, ETH_TYPE_IP4]))
        msg.append(makeIp4Hdr(self.myIp4Addr, tgt_ip, self.ip4TxCount, IP4_TYPE_UDP, 8 + len(data)))
        self.ip4TxCount += 1
        msg.append(makeUdp4Hdr(self.myIp4Addr, src_port, tgt_ip, tgt_port, data))
        msg.append(data)
        n = self.txPkt(msg)
        return n

    def sendUdp4Bcast(self, tgt_port, src_port, data, src_ip4Addr=None):
        msg = []
        tgt_ip4Addr = IP4_ADDR_BCAST
        if None == src_ip4Addr:
            src_ip4Addr = IP4_ADDR_ZERO
        msg.append(bytearray([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]))
        msg.append(self.myMacAddr)
        msg.append(bytearray([ETH_TYPE_IP4 >> 8, ETH_TYPE_IP4]))
        msg.append(makeIp4Hdr(src_ip4Addr, tgt_ip4Addr, self.ip4TxCount, IP4_TYPE_UDP, 8 + len(data)))
        self.ip4TxCount += 1
        msg.append(makeUdp4Hdr(src_ip4Addr, src_port, tgt_ip4Addr, tgt_port, data))
        msg.append(data)
        n = self.txPkt(msg)
        return n


class Udp4EchoServer:
    '''Simple UDP Echo server'''
    def __init__(self, ntw):
        self.ntw = ntw

    def __call__(self, pkt):
        print(f'Rx UDP Echo req from IP {pkt.ip_src_addr[0]}.{pkt.ip_src_addr[1]}.{pkt.ip_src_addr[2]}.{pkt.ip_src_addr[3]}')
        pkt.ntw.addArpEntry(pkt.ip_src_addr, pkt.eth_src)
        pkt.ntw.sendUdp4(pkt.ip_src_addr, pkt.udp_srcPort, pkt.udp_data, pkt.udp_dstPort)


def main():
    ntw = Ntw()

    # Set static IP address
    ntw.setIPv4([192,168,40,233], [255,255,255,0], [192,168,40,1])

    # Create UDP Echo server
    udpecho = Udp4EchoServer(ntw)

    # Bind UDP Echo server to UDP port 7
    ntw.registerUdp4Callback(7, udpecho)

    while True:
        ntw.rxAllPkt()


if __name__ == '__main__':
    main()
