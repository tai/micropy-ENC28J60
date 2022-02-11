#!/usr/bin/env python
# -*- coding: utf8 -*-

#   Port from C to Python by Przemyslaw Bereski https://github.com/przemobe/
#   based on https://www.oryx-embedded.com/doc/enc28j60__driver_8c_source.html
#
#   This implementation is for MicroPython v1.17 (RP2)
#
#   This program is free software; you can redistribute it and/or
#   modify it under the terms of the GNU General Public License
#   as published by the Free Software Foundation; either version 2
#   of the License, or (at your option) any later version.


from micropython import const
from machine import Pin
from machine import SPI
from machine import unique_id
import time
import struct

# RX buffer size
ETH_RX_BUFFER_SIZE          = 1536

# RX error codes
ETH_RX_ERR_UNSPECIFIED      = -1

# TX buffer size
ETH_TX_BUFFER_SIZE          = 1536

# TX error codes
ETH_TX_ERR_MSGSIZE          = -1
ETH_TX_ERR_LINKDOWN         = -2

# Receive and transmit buffers
RX_BUFFER_START             = 0x0000
RX_BUFFER_STOP              = 0x17FF
TX_BUFFER_START             = 0x1800
TX_BUFFER_STOP              = 0x1FFF

# SPI command set
CMD_RCR                     = 0x00
CMD_RBM                     = 0x3A
CMD_WCR                     = 0x40
CMD_WBM                     = 0x7A
CMD_BFS                     = 0x80
CMD_BFC                     = 0xA0
CMD_SRC                     = 0xFF

# ENC28J60 register types
ETH_REG_TYPE                         = 0x0000
MAC_REG_TYPE                         = 0x1000
MII_REG_TYPE                         = 0x2000
PHY_REG_TYPE                         = 0x3000

# ENC28J60 banks
BANK_0                               = 0x00
BANK_1                               = 0x20
BANK_2                               = 0x40
BANK_3                               = 0x60

# Related masks
REG_BANK_MASK                        = 0x60
REG_ADDR_MASK                        = 0x1F

# ENC28J60 registers
ERDPTL                      = (0x00)
ERDPTH                      = (0x01)
EWRPTL                      = (0x02)
EWRPTH                      = (0x03)
ETXSTL                      = (0x04)
ETXSTH                      = (0x05)
ETXNDL                      = (0x06)
ETXNDH                      = (0x07)
ERXSTL                      = (0x08)
ERXSTH                      = (0x09)
ERXNDL                      = (0x0A)
ERXNDH                      = (0x0B)
ERXRDPTL                    = (0x0C)
ERXRDPTH                    = (0x0D)
ERXWRPTL                    = (0x0E)
ERXWRPTH                    = (0x0F)
EDMASTL                     = (0x10)
EDMASTH                     = (0x11)
EDMANDL                     = (0x12)
EDMANDH                     = (0x13)
EDMADSTL                    = (0x14)
EDMADSTH                    = (0x15)
EDMACSL                     = (0x16)
EDMACSH                     = (0x17)
EIE                         = (0x1B)
EIR                         = (0x1C)
ESTAT                       = (0x1D)
ECON2                       = (0x1E)
ECON1                       = (0x1F)
EHT0                        = (BANK_1 | 0x00)
EHT1                        = (BANK_1 | 0x01)
EHT2                        = (BANK_1 | 0x02)
EHT3                        = (BANK_1 | 0x03)
EHT4                        = (BANK_1 | 0x04)
EHT5                        = (BANK_1 | 0x05)
EHT6                        = (BANK_1 | 0x06)
EHT7                        = (BANK_1 | 0x07)
EPMM0                       = (BANK_1 | 0x08)
EPMM1                       = (BANK_1 | 0x09)
EPMM2                       = (BANK_1 | 0x0A)
EPMM3                       = (BANK_1 | 0x0B)
EPMM4                       = (BANK_1 | 0x0C)
EPMM5                       = (BANK_1 | 0x0D)
EPMM6                       = (BANK_1 | 0x0E)
EPMM7                       = (BANK_1 | 0x0F)
EPMCSL                      = (BANK_1 | 0x10)
EPMCSH                      = (BANK_1 | 0x11)
EPMOL                       = (BANK_1 | 0x14)
EPMOH                       = (BANK_1 | 0x15)
EWOLIE                      = (BANK_1 | 0x16)
EWOLIR                      = (BANK_1 | 0x17)
ERXFCON                     = (BANK_1 | 0x18)
EPKTCNT                     = (BANK_1 | 0x19)
MACON1                      = (BANK_2 | 0x00)
MACON2                      = (BANK_2 | 0x01)
MACON3                      = (BANK_2 | 0x02)
MACON4                      = (BANK_2 | 0x03)
MABBIPG                     = (BANK_2 | 0x04)
MAIPGL                      = (BANK_2 | 0x06)
MAIPGH                      = (BANK_2 | 0x07)
MACLCON1                    = (BANK_2 | 0x08)
MACLCON2                    = (BANK_2 | 0x09)
MAMXFLL                     = (BANK_2 | 0x0A)
MAMXFLH                     = (BANK_2 | 0x0B)
MAPHSUP                     = (BANK_2 | 0x0D)
MICON                       = (BANK_2 | 0x11)
MICMD                       = (BANK_2 | 0x12)
MIREGADR                    = (BANK_2 | 0x14)
MIWRL                       = (BANK_2 | 0x16)
MIWRH                       = (BANK_2 | 0x17)
MIRDL                       = (BANK_2 | 0x18)
MIRDH                       = (BANK_2 | 0x19)
MAADR1                      = (BANK_3 | 0x00)
MAADR0                      = (BANK_3 | 0x01)
MAADR3                      = (BANK_3 | 0x02)
MAADR2                      = (BANK_3 | 0x03)
MAADR5                      = (BANK_3 | 0x04)
MAADR4                      = (BANK_3 | 0x05)
EBSTSD                      = (BANK_3 | 0x06)
EBSTCON                     = (BANK_3 | 0x07)
EBSTCSL                     = (BANK_3 | 0x08)
EBSTCSH                     = (BANK_3 | 0x09)
MISTAT                      = (BANK_3 | 0x0A)
EREVID                      = (BANK_3 | 0x12)
ECOCON                      = (BANK_3 | 0x15)
EFLOCON                     = (BANK_3 | 0x17)
EPAUSL                      = (BANK_3 | 0x18)
EPAUSH                      = (BANK_3 | 0x19)

# ENC28J60 PHY registers
PHCON1                      = (0x00)
PHSTAT1                     = (0x01)
PHID1                       = (0x02)
PHID2                       = (0x03)
PHCON2                      = (0x10)
PHSTAT2                     = (0x11)
PHIE                        = (0x12)
PHIR                        = (0x13)
PHLCON                      = (0x14)

# Ethernet Interrupt Enable register
EIE_INTIE                   = 0x80
EIE_PKTIE                   = 0x40
EIE_DMAIE                   = 0x20
EIE_LINKIE                  = 0x10
EIE_TXIE                    = 0x08
EIE_WOLIE                   = 0x04
EIE_TXERIE                  = 0x02
EIE_RXERIE                  = 0x01

# Ethernet Interrupt Request register
EIR_PKTIF                   = 0x40
EIR_DMAIF                   = 0x20
EIR_LINKIF                  = 0x10
EIR_TXIF                    = 0x08
EIR_WOLIF                   = 0x04
EIR_TXERIF                  = 0x02
EIR_RXERIF                  = 0x01

# Ethernet Status register
ESTAT_INT                   = 0x80
ESTAT_R6                    = 0x40
ESTAT_R5                    = 0x20
ESTAT_LATECOL               = 0x10
ESTAT_RXBUSY                = 0x04
ESTAT_TXABRT                = 0x02
ESTAT_CLKRDY                = 0x01

# Ethernet Control 2 register
ECON2_AUTOINC               = 0x80
ECON2_PKTDEC                = 0x40
ECON2_PWRSV                 = 0x20
ECON2_VRPS                  = 0x08

# Ethernet Control 1 register
ECON1_TXRST                 = 0x80
ECON1_RXRST                 = 0x40
ECON1_DMAST                 = 0x20
ECON1_CSUMEN                = 0x10
ECON1_TXRTS                 = 0x08
ECON1_RXEN                  = 0x04
ECON1_BSEL1                 = 0x02
ECON1_BSEL0                 = 0x01

# Ethernet Wake-Up On LAN Interrupt Enable register
EWOLIE_UCWOLIE              = 0x80
EWOLIE_AWOLIE               = 0x40
EWOLIE_PMWOLIE              = 0x10
EWOLIE_MPWOLIE              = 0x08
EWOLIE_HTWOLIE              = 0x04
EWOLIE_MCWOLIE              = 0x02
EWOLIE_BCWOLIE              = 0x01

# Ethernet Wake-Up On LAN Interrupt Request register
EWOLIR_UCWOLIF              = 0x80
EWOLIR_AWOLIF               = 0x40
EWOLIR_PMWOLIF              = 0x10
EWOLIR_MPWOLIF              = 0x08
EWOLIR_HTWOLIF              = 0x04
EWOLIR_MCWOLIF              = 0x02
EWOLIR_BCWOLIF              = 0x01

# Receive Filter Control register
ERXFCON_UCEN                = 0x80
ERXFCON_ANDOR               = 0x40
ERXFCON_CRCEN               = 0x20
ERXFCON_PMEN                = 0x10
ERXFCON_MPEN                = 0x08
ERXFCON_HTEN                = 0x04
ERXFCON_MCEN                = 0x02
ERXFCON_BCEN                = 0x01

# MAC Control 1 register
MACON1_LOOPBK               = 0x10
MACON1_TXPAUS               = 0x08
MACON1_RXPAUS               = 0x04
MACON1_PASSALL              = 0x02
MACON1_MARXEN               = 0x01

# MAC Control 2 register
MACON2_MARST                = 0x80
MACON2_RNDRST               = 0x40
MACON2_MARXRST              = 0x08
MACON2_RFUNRST              = 0x04
MACON2_MATXRST              = 0x02
MACON2_TFUNRST              = 0x01

# MAC Control 3 register
MACON3_PADCFG               = 0xE0
MACON3_PADCFG_NO            = 0x00
MACON3_PADCFG_60_BYTES      = 0x20
MACON3_PADCFG_64_BYTES      = 0x60
MACON3_PADCFG_AUTO          = 0xA0
MACON3_TXCRCEN              = 0x10
MACON3_PHDRLEN              = 0x08
MACON3_HFRMEN               = 0x04
MACON3_FRMLNEN              = 0x02
MACON3_FULDPX               = 0x01

# MAC Control 4 register
MACON4_DEFER                = 0x40
MACON4_BPEN                 = 0x20
MACON4_NOBKOFF              = 0x10
MACON4_LONGPRE              = 0x02
MACON4_PUREPRE              = 0x01

# Back-to-Back Inter-Packet Gap register
MABBIPG_DEFAULT_HD          = 0x12
MABBIPG_DEFAULT_FD          = 0x15

# Non-Back-to-Back Inter-Packet Gap Low Byte register
MAIPGL_DEFAULT              = 0x12

# Non-Back-to-Back Inter-Packet Gap High Byte register
MAIPGH_DEFAULT              = 0x0C

# Retransmission Maximum register
MACLCON1_RETMAX             = 0x0F

# Collision Window register
MACLCON2_COLWIN             = 0x3F
MACLCON2_COLWIN_DEFAULT     = 0x37

# MAC-PHY Support register
MAPHSUP_RSTINTFC            = 0x80
MAPHSUP_R4                  = 0x10
MAPHSUP_RSTRMII             = 0x08
MAPHSUP_R0                  = 0x01

# MII Control register
MICON_RSTMII                = 0x80

# MII Command register
MICMD_MIISCAN               = 0x02
MICMD_MIIRD                 = 0x01

# MII Register Address register
MIREGADR_VAL                = 0x1F

# Self-Test Control register
EBSTCON_PSV                 = 0xE0
EBSTCON_PSEL                = 0x10
EBSTCON_TMSEL               = 0x0C
EBSTCON_TMSEL_RANDOM        = 0x00
EBSTCON_TMSEL_ADDR          = 0x04
EBSTCON_TMSEL_PATTERN_SHIFT = 0x08
EBSTCON_TMSEL_RACE_MODE     = 0x0C
EBSTCON_TME                 = 0x02
EBSTCON_BISTST              = 0x01

# MII Status register
MISTAT_R3                   = 0x08
MISTAT_NVALID               = 0x04
MISTAT_SCAN                 = 0x02
MISTAT_BUSY                 = 0x01

# Ethernet Revision ID register
EREVID_REV                  = 0x1F
EREVID_REV_B1               = 0x02
EREVID_REV_B4               = 0x04
EREVID_REV_B5               = 0x05
EREVID_REV_B7               = 0x06

# Clock Output Control register
ECOCON_COCON                = 0x07
ECOCON_COCON_DISABLED       = 0x00
ECOCON_COCON_DIV1           = 0x01
ECOCON_COCON_DIV2           = 0x02
ECOCON_COCON_DIV3           = 0x03
ECOCON_COCON_DIV4           = 0x04
ECOCON_COCON_DIV8           = 0x05

# Ethernet Flow Control register
EFLOCON_FULDPXS             = 0x04
EFLOCON_FCEN                = 0x03
EFLOCON_FCEN_OFF            = 0x00
EFLOCON_FCEN_ON_HD          = 0x01
EFLOCON_FCEN_ON_FD          = 0x02
EFLOCON_FCEN_SEND_PAUSE     = 0x03

# PHY Control 1 register
PHCON1_PRST                 = 0x8000
PHCON1_PLOOPBK              = 0x4000
PHCON1_PPWRSV               = 0x0800
PHCON1_PDPXMD               = 0x0100

# Physical Layer Status 1 register
PHSTAT1_PFDPX               = 0x1000
PHSTAT1_PHDPX               = 0x0800
PHSTAT1_LLSTAT              = 0x0004
PHSTAT1_JBRSTAT             = 0x0002

# PHY Identifier 1 register
PHID1_PIDH                  = 0xFFFF
PHID1_PIDH_DEFAULT          = 0x0083

# PHY Identifier 2 register
PHID2_PIDL                  = 0xFC00
PHID2_PIDL_DEFAULT          = 0x1400
PHID2_PPN                   = 0x03F0
PHID2_PPN_DEFAULT           = 0x0000
PHID2_PREV                  = 0x000F

# PHY Control 2 register
PHCON2_FRCLNK               = 0x4000
PHCON2_TXDIS                = 0x2000
PHCON2_JABBER               = 0x0400
PHCON2_HDLDIS               = 0x0100

# Physical Layer Status 2 register
PHSTAT2_TXSTAT              = 0x2000
PHSTAT2_RXSTAT              = 0x1000
PHSTAT2_COLSTAT             = 0x0800
PHSTAT2_LSTAT               = 0x0400
PHSTAT2_DPXSTAT             = 0x0200
PHSTAT2_PLRITY              = 0x0010

# PHY Interrupt Enable register
PHIE_PLNKIE                 = 0x0010
PHIE_PGEIE                  = 0x0002

# PHY Interrupt Request register
PHIR_PLNKIF                 = 0x0010
PHIR_PGIF                   = 0x0004

# PHY Module LED Control register
PHLCON_LACFG                = 0x0F00
PHLCON_LACFG_TX             = 0x0100
PHLCON_LACFG_RX             = 0x0200
PHLCON_LACFG_COL            = 0x0300
PHLCON_LACFG_LINK           = 0x0400
PHLCON_LACFG_DUPLEX         = 0x0500
PHLCON_LACFG_TX_RX          = 0x0700
PHLCON_LACFG_ON             = 0x0800
PHLCON_LACFG_OFF            = 0x0900
PHLCON_LACFG_BLINK_FAST     = 0x0A00
PHLCON_LACFG_BLINK_SLOW     = 0x0B00
PHLCON_LACFG_LINK_RX        = 0x0C00
PHLCON_LACFG_LINK_TX_RX     = 0x0D00
PHLCON_LACFG_DUPLEX_COL     = 0x0E00
PHLCON_LBCFG                = 0x00F0
PHLCON_LBCFG_TX             = 0x0010
PHLCON_LBCFG_RX             = 0x0020
PHLCON_LBCFG_COL            = 0x0030
PHLCON_LBCFG_LINK           = 0x0040
PHLCON_LBCFG_DUPLEX         = 0x0050
PHLCON_LBCFG_TX_RX          = 0x0070
PHLCON_LBCFG_ON             = 0x0080
PHLCON_LBCFG_OFF            = 0x0090
PHLCON_LBCFG_BLINK_FAST     = 0x00A0
PHLCON_LBCFG_BLINK_SLOW     = 0x00B0
PHLCON_LBCFG_LINK_RX        = 0x00C0
PHLCON_LBCFG_LINK_TX_RX     = 0x00D0
PHLCON_LBCFG_DUPLEX_COL     = 0x00E0
PHLCON_LFRQ                 = 0x000C
PHLCON_LFRQ_40_MS           = 0x0000
PHLCON_LFRQ_73_MS           = 0x0004
PHLCON_LFRQ_139_MS          = 0x0008
PHLCON_STRCH                = 0x0002

# Per-packet control byte
TX_CTRL_PHUGEEN             = 0x08
TX_CTRL_PPADEN              = 0x04
TX_CTRL_PCRCEN              = 0x02
TX_CTRL_POVERRIDE           = 0x01

# Receive status vector
RSV_VLAN_TYPE               = 0x4000
RSV_UNKNOWN_OPCODE          = 0x2000
RSV_PAUSE_CONTROL_FRAME     = 0x1000
RSV_CONTROL_FRAME           = 0x0800
RSV_DRIBBLE_NIBBLE          = 0x0400
RSV_BROADCAST_PACKET        = 0x0200
RSV_MULTICAST_PACKET        = 0x0100
RSV_RECEIVED_OK             = 0x0080
RSV_LENGTH_OUT_OF_RANGE     = 0x0040
RSV_LENGTH_CHECK_ERROR      = 0x0020
RSV_CRC_ERROR               = 0x0010
RSV_CARRIER_EVENT           = 0x0004
RSV_DROP_EVENT              = 0x0001

def LSB(val):
    return (val & 0xFF)

def MSB(val):
    return ((val >> 8) & 0xFF)


class ENC28J60:
    '''
    This class provides control over ENC28J60 Ethernet chips.
    '''

    def __init__(self, spi, cs, macAddr = None, fullDuplex = True):
        self.fullDuplex = fullDuplex
        self.revId = None
        self.tmpBytearray1B = bytearray(1)
        self.tmpBytearray2B = bytearray(2)
        self.tmpBytearray3B = bytearray(3)
        self.tmpBytearray6B = bytearray(6)

        # SPI
        self.spi = spi
        self.spi.init()

        # MAC Address
        if macAddr:
            self.macAddr = bytearray(macAddr)
        else:
            self.macAddr = bytearray(b'\x0e\x5f\x5f' + unique_id()[-3:])

        # PIN CS
        self.cs = cs
        self.cs.init(Pin.OUT, value=1)

        #self.init()

    def getMacAddr(self):
        return self.macAddr

    def init(self):
        # Issue a system reset
        self.SoftReset()

        # After issuing the reset command, wait at least 1ms in firmware for the device to be ready
        time.sleep_ms(10)

        # Initialize driver specific variables
        self.currentBank = 0xFFFF
        self.nextPacket = RX_BUFFER_START

        # Read silicon revision ID
        self.revId = self.ReadRegE(EREVID) & EREVID_REV

        # Disable CLKOUT output
        self.WriteReg(ECOCON, ECOCON_COCON_DISABLED)

        # Set the MAC address of the station
        self.WriteReg(MAADR5, self.macAddr[0])
        self.WriteReg(MAADR4, self.macAddr[1])
        self.WriteReg(MAADR3, self.macAddr[2])
        self.WriteReg(MAADR2, self.macAddr[3])
        self.WriteReg(MAADR1, self.macAddr[4])
        self.WriteReg(MAADR0, self.macAddr[5])

        # Set receive buffer location
        self.WriteReg(ERXSTL, LSB(RX_BUFFER_START))
        self.WriteReg(ERXSTH, MSB(RX_BUFFER_START))
        self.WriteReg(ERXNDL, LSB(RX_BUFFER_STOP))
        self.WriteReg(ERXNDH, MSB(RX_BUFFER_STOP))

        # The ERXRDPT register defines a location within the FIFO where the receive hardware is forbidden to write to
        self.WriteReg(ERXRDPTL, LSB(RX_BUFFER_STOP))
        self.WriteReg(ERXRDPTH, MSB(RX_BUFFER_STOP))

        # Configure the receive filters
        self.WriteReg(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_HTEN | ERXFCON_BCEN)

        # Initialize the hash table
        self.WriteReg(EHT0, 0x00)
        self.WriteReg(EHT1, 0x00)
        self.WriteReg(EHT2, 0x00)
        self.WriteReg(EHT3, 0x00)
        self.WriteReg(EHT4, 0x00)
        self.WriteReg(EHT5, 0x00)
        self.WriteReg(EHT6, 0x00)
        self.WriteReg(EHT7, 0x00)

        # Pull the MAC out of reset
        self.WriteReg(MACON2, 0x00)

        # Enable the MAC to receive frames
        self.WriteReg(MACON1, MACON1_TXPAUS | MACON1_RXPAUS | MACON1_MARXEN)

        # Enable automatic padding, always append a valid CRC and check frame length. MAC can operate in half-duplex or full-duplex mode
        if self.fullDuplex:
            self.WriteReg(MACON3, MACON3_PADCFG_AUTO | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX)
        else:
            self.WriteReg(MACON3, MACON3_PADCFG_AUTO | MACON3_TXCRCEN | MACON3_FRMLNEN)

        # When the medium is occupied, the MAC will wait indefinitely for it to become free when attempting to transmit
        self.WriteReg(MACON4, MACON4_DEFER)

        # Maximum frame length that can be received or transmitted
        self.WriteReg(MAMXFLL, LSB(ETH_RX_BUFFER_SIZE))
        self.WriteReg(MAMXFLH, MSB(ETH_RX_BUFFER_SIZE))

        # Configure the back-to-back inter-packet gap register
        if self.fullDuplex:
            self.WriteReg(MABBIPG, MABBIPG_DEFAULT_FD)
        else:
            self.WriteReg(MABBIPG, MABBIPG_DEFAULT_HD)

        # Configure the non-back-to-back inter-packet gap register
        self.WriteReg(MAIPGL, MAIPGL_DEFAULT)
        self.WriteReg(MAIPGH, MAIPGH_DEFAULT)

        # Collision window register
        self.WriteReg(MACLCON2, MACLCON2_COLWIN_DEFAULT)

        # Set the PHY to the proper duplex mode
        if self.fullDuplex:
            self.WritePhyReg(PHCON1, PHCON1_PDPXMD)
        else:
            self.WritePhyReg(PHCON1, 0x0000)

        # Disable half-duplex loopback in PHY
        self.WritePhyReg(PHCON2, PHCON2_HDLDIS)

        # LEDA displays link status and LEDB displays TX/RX activity
        #self.WritePhyReg(PHLCON, PHLCON_LACFG_LINK | PHLCON_LBCFG_TX_RX | PHLCON_LFRQ_40_MS | PHLCON_STRCH)

        # Clear interrupt flags
        self.WriteReg(EIR, 0x00)

        # Configure interrupts as desired
        self.WriteReg(EIE, EIE_INTIE | EIE_PKTIE | EIE_LINKIE)
        # | EIE_TXIE | EIE_TXERIE)

        # Configure PHY interrupts as desired
        self.WritePhyReg(PHIE, PHIE_PLNKIE | PHIE_PGEIE)

        # Set RXEN to enable reception
        self.WriteReg(ECON1, ECON1_RXEN)

    def writeSpi(self, data):
        self.cs(0)
        self.spi.write(data)
        self.cs(1)

    def SoftReset(self):
        self.tmpBytearray1B[0] = CMD_SRC
        self.writeSpi(self.tmpBytearray1B)

    def ClearBit(self, address, mask):
        self.tmpBytearray2B[0] = (CMD_BFC | (address & REG_ADDR_MASK))
        self.tmpBytearray2B[1] = mask
        self.writeSpi(self.tmpBytearray2B)

    def SetBit(self, address, mask):
        self.tmpBytearray2B[0] = (CMD_BFS | (address & REG_ADDR_MASK))
        self.tmpBytearray2B[1] = mask
        self.writeSpi(self.tmpBytearray2B)

    def SelectBank(self, address):
        # uint16_t address
        bank = address & REG_BANK_MASK

        # Rewrite the bank number only if a change is detected
        if (bank == self.currentBank):
            return

        # Select the relevant bank
        if bank == BANK_0:
            self.ClearBit(ECON1, ECON1_BSEL1 | ECON1_BSEL0)
        elif bank == BANK_1:
            self.SetBit(ECON1, ECON1_BSEL0)
            self.ClearBit(ECON1, ECON1_BSEL1)
        elif bank == BANK_2:
            self.ClearBit(ECON1, ECON1_BSEL0)
            self.SetBit(ECON1, ECON1_BSEL1)
        else:
            self.SetBit(ECON1, ECON1_BSEL1 | ECON1_BSEL0)

        # Save bank number
        self.currentBank = bank
        return

    def WriteReg(self, address, data):
        # Make sure the corresponding bank is selected
        self.SelectBank(address)

        # Write opcode and register address, Write register value
        self.tmpBytearray2B[0] = (CMD_WCR | (address & REG_ADDR_MASK))
        self.tmpBytearray2B[1] = data
        self.writeSpi(self.tmpBytearray2B)
        return

    def ReadRegE(self, address):
        return self.ReadReg(address)

    def ReadRegMI(self, address):
        return self.ReadReg(address, True)

    def ReadRegMA(self, address):
        return self.ReadReg(address, True)

    def ReadReg(self, address, need_dummy=False):
        # Make sure the corresponding bank is selected
        self.SelectBank(address)

        # Pull the CS pin low
        self.cs(0)

        # Write opcode and register address
        self.spi.write(bytearray([CMD_RCR | (address & REG_ADDR_MASK)]))

        # When reading MAC or MII registers, a dummy byte is first shifted out
        if need_dummy:
            self.spi.write(self.tmpBytearray1B)

        # Read register contents
        self.spi.readinto(self.tmpBytearray1B)

        # Terminate the operation by raising the CS pin
        self.cs(1)

        # Return register contents
        return self.tmpBytearray1B[0]

    def WritePhyReg(self, address, data):
        # Write register address
        self.WriteReg(MIREGADR, address & REG_ADDR_MASK)

        # Write the lower 8 bits
        self.WriteReg(MIWRL, LSB(data))
        # Write the upper 8 bits
        self.WriteReg(MIWRH, MSB(data))

        # Wait until the PHY register has been written
        while 0 != (self.ReadRegMI(MISTAT) & MISTAT_BUSY):
            pass
        return

    def ReadPhyReg(self, address):
        # Write register address
        self.WriteReg(MIREGADR, address & REG_ADDR_MASK)

        # Start read operation
        self.WriteReg(MICMD, MICMD_MIIRD)

        # Wait for the read operation to complete
        while 0 != (self.ReadRegMI(MISTAT) & MISTAT_BUSY):
            pass

        # Clear command register
        self.WriteReg(MICMD, 0)

        # Read the lower 8 bits
        data = self.ReadRegMI(MIRDL)
        # Read the upper 8 bits
        data |= self.ReadRegMI(MIRDH) << 8

        # Return register contents
        return data

    def WriteBuffer(self, chunks):
        # Pull the CS pin low
        self.cs(0)

        # Write opcode, Write per-packet control byte
        self.tmpBytearray2B[0] = CMD_WBM
        self.tmpBytearray2B[1] = 0x00
        self.spi.write(self.tmpBytearray2B)

        # Loop through data chunks
        for data in chunks:
            self.spi.write(data)

        # Terminate the operation by raising the CS pin
        self.cs(1)

    def ReadBuffer(self, data):
        # Pull the CS pin low
        self.cs(0)

        # Write opcode
        self.tmpBytearray1B[0] = CMD_RBM
        self.spi.write(self.tmpBytearray1B)

        # Copy data from SRAM buffer
        self.spi.readinto(data)

        # Terminate the operation by raising the CS pin
        self.cs(1)

    def GetRevId(self):
        if None == self.revId:
            self.revId = self.ReadRegE(EREVID) & EREVID_REV
        return self.revId

    def IsLinkUp(self):
        return 0 != (self.ReadPhyReg(PHSTAT2) & PHSTAT2_LSTAT)

    def GetRxPacketCnt(self):
        return self.ReadRegE(EPKTCNT)

    def SendPacket(self, chunks):
        # Retrieve the length of the packet
        length = 0
        for data in chunks:
            length += len(data)

        # Check the frame length
        if length > ETH_TX_BUFFER_SIZE:
            return ETH_TX_ERR_MSGSIZE

        # Make sure the link is up before transmitting the frame
        if False == self.IsLinkUp():
            return ETH_TX_ERR_LINKDOWN

        # It is recommended to reset the transmit logic before attempting to transmit a packet
        self.SetBit(ECON1, ECON1_TXRST)
        self.ClearBit(ECON1, ECON1_TXRST)

        # Interrupt flags should be cleared after the reset is completed
        self.ClearBit(EIR, EIR_TXIF | EIR_TXERIF)

        # Set transmit buffer location
        self.WriteReg(ETXSTL, LSB(TX_BUFFER_START))
        self.WriteReg(ETXSTH, MSB(TX_BUFFER_START))

        # Point to start of transmit buffer
        self.WriteReg(EWRPTL, LSB(TX_BUFFER_START))
        self.WriteReg(EWRPTH, MSB(TX_BUFFER_START))

        # Copy the data to the transmit buffer
        self.WriteBuffer(chunks)

        # ETXND should point to the last byte in the data payload
        self.WriteReg(ETXNDL, LSB(TX_BUFFER_START + length))
        self.WriteReg(ETXNDH, MSB(TX_BUFFER_START + length))

        # Start transmission
        self.SetBit(ECON1, ECON1_TXRTS)
        return length

    def ReceivePacket(self, rxBuffer):
        if 0 == self.GetRxPacketCnt():
            return 0

        # Point to the start of the received packet
        self.WriteReg(ERDPTL, LSB(self.nextPacket))
        self.WriteReg(ERDPTH, MSB(self.nextPacket))

        # The packet is preceded by a 6-byte header
        self.ReadBuffer(self.tmpBytearray6B)

        # Unpack header, little-endian
        headerStruct = struct.unpack("<HHH", self.tmpBytearray6B)

        # The first two bytes are the address of the next packet
        self.nextPacket = headerStruct[0]

        # Get the length of the received packet
        length = headerStruct[1]

        # Get the receive status vector (RSV)
        status = headerStruct[2]

        # Make sure no error occurred
        if 0 != (status & RSV_RECEIVED_OK):
            # Limit the number of data to read
            length = min(length, ETH_RX_BUFFER_SIZE)
            length = min(length, len(rxBuffer))

            # Read the Ethernet frame
            self.ReadBuffer(memoryview(rxBuffer)[0:length])
        else:
            # The received packet contains an error
            length = ETH_RX_ERR_UNSPECIFIED

        # Advance the ERXRDPT pointer, taking care to wrap back at the end of the received memory buffer
        if RX_BUFFER_START == self.nextPacket:
            self.WriteReg(ERXRDPTL, LSB(RX_BUFFER_STOP))
            self.WriteReg(ERXRDPTH, MSB(RX_BUFFER_STOP))
        else:
            self.WriteReg(ERXRDPTL, LSB(self.nextPacket - 1))
            self.WriteReg(ERXRDPTH, MSB(self.nextPacket - 1))

        # Decrement the packet counter
        self.SetBit(ECON2, ECON2_PKTDEC)
        return length
