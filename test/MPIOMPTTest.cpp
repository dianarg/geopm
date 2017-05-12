/*
 * Copyright (c) 2015, 2016, 2017, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <fstream>
#include "gtest/gtest.h"
#include "OMPT.hpp"

static const char *g_test_map = 
"00400000-005a8000 r-xp 00000000 fd:00 101506074                          /usr/bin/emacs-24.3-nox\n"
"007a7000-007a8000 r--p 001a7000 fd:00 101506074                          /usr/bin/emacs-24.3-nox\n"
"007a8000-012b4000 rw-p 001a8000 fd:00 101506074                          /usr/bin/emacs-24.3-nox\n"
"0158e000-0168f000 rw-p 00000000 00:00 0                                  [heap]\n"
"7f4a149ae000-7f4a149b9000 r-xp 00000000 fd:00 251660409                  /usr/lib64/libnss_files-2.17.so\n"
"7f4a149b9000-7f4a14bb8000 ---p 0000b000 fd:00 251660409                  /usr/lib64/libnss_files-2.17.so\n"
"7f4a14bb8000-7f4a14bb9000 r--p 0000a000 fd:00 251660409                  /usr/lib64/libnss_files-2.17.so\n"
"7f4a14bb9000-7f4a14bba000 rw-p 0000b000 fd:00 251660409                  /usr/lib64/libnss_files-2.17.so\n"
"7f4a14bba000-7f4a14bc0000 rw-p 00000000 00:00 0 \n"
"7f4a14bc0000-7f4a1b0e7000 r--p 00000000 fd:00 50332726                   /usr/lib/locale/locale-archive\n"
"7f4a1b0e7000-7f4a1b0fd000 r-xp 00000000 fd:00 251663875                  /usr/lib64/libresolv-2.17.so\n"
"7f4a1b0fd000-7f4a1b2fd000 ---p 00016000 fd:00 251663875                  /usr/lib64/libresolv-2.17.so\n"
"7f4a1b2fd000-7f4a1b2fe000 r--p 00016000 fd:00 251663875                  /usr/lib64/libresolv-2.17.so\n"
"7f4a1b2fe000-7f4a1b2ff000 rw-p 00017000 fd:00 251663875                  /usr/lib64/libresolv-2.17.so\n"
"7f4a1b2ff000-7f4a1b301000 rw-p 00000000 00:00 0 \n"
"7f4a1b301000-7f4a1b304000 r-xp 00000000 fd:00 251770394                  /usr/lib64/libkeyutils.so.1.5\n"
"7f4a1b304000-7f4a1b503000 ---p 00003000 fd:00 251770394                  /usr/lib64/libkeyutils.so.1.5\n"
"7f4a1b503000-7f4a1b504000 r--p 00002000 fd:00 251770394                  /usr/lib64/libkeyutils.so.1.5\n"
"7f4a1b504000-7f4a1b505000 rw-p 00003000 fd:00 251770394                  /usr/lib64/libkeyutils.so.1.5\n"
"7f4a1b505000-7f4a1b512000 r-xp 00000000 fd:00 251826717                  /usr/lib64/libkrb5support.so.0.1\n"
"7f4a1b512000-7f4a1b712000 ---p 0000d000 fd:00 251826717                  /usr/lib64/libkrb5support.so.0.1\n"
"7f4a1b712000-7f4a1b713000 r--p 0000d000 fd:00 251826717                  /usr/lib64/libkrb5support.so.0.1\n"
"7f4a1b713000-7f4a1b714000 rw-p 0000e000 fd:00 251826717                  /usr/lib64/libkrb5support.so.0.1\n"
"7f4a1b714000-7f4a1b743000 r-xp 00000000 fd:00 251826705                  /usr/lib64/libk5crypto.so.3.1\n"
"7f4a1b743000-7f4a1b942000 ---p 0002f000 fd:00 251826705                  /usr/lib64/libk5crypto.so.3.1\n"
"7f4a1b942000-7f4a1b944000 r--p 0002e000 fd:00 251826705                  /usr/lib64/libk5crypto.so.3.1\n"
"7f4a1b944000-7f4a1b945000 rw-p 00030000 fd:00 251826705                  /usr/lib64/libk5crypto.so.3.1\n"
"7f4a1b945000-7f4a1b946000 rw-p 00000000 00:00 0 \n"
"7f4a1b946000-7f4a1b949000 r-xp 00000000 fd:00 251666982                  /usr/lib64/libcom_err.so.2.1\n"
"7f4a1b949000-7f4a1bb48000 ---p 00003000 fd:00 251666982                  /usr/lib64/libcom_err.so.2.1\n"
"7f4a1bb48000-7f4a1bb49000 r--p 00002000 fd:00 251666982                  /usr/lib64/libcom_err.so.2.1\n"
"7f4a1bb49000-7f4a1bb4a000 rw-p 00003000 fd:00 251666982                  /usr/lib64/libcom_err.so.2.1\n"
"7f4a1bb4a000-7f4a1bc1f000 r-xp 00000000 fd:00 251826715                  /usr/lib64/libkrb5.so.3.3\n"
"7f4a1bc1f000-7f4a1be1f000 ---p 000d5000 fd:00 251826715                  /usr/lib64/libkrb5.so.3.3\n"
"7f4a1be1f000-7f4a1be2c000 r--p 000d5000 fd:00 251826715                  /usr/lib64/libkrb5.so.3.3\n"
"7f4a1be2c000-7f4a1be2f000 rw-p 000e2000 fd:00 251826715                  /usr/lib64/libkrb5.so.3.3\n"
"7f4a1be2f000-7f4a1be78000 r-xp 00000000 fd:00 251826702                  /usr/lib64/libgssapi_krb5.so.2.2\n"
"7f4a1be78000-7f4a1c078000 ---p 00049000 fd:00 251826702                  /usr/lib64/libgssapi_krb5.so.2.2\n"
"7f4a1c078000-7f4a1c079000 r--p 00049000 fd:00 251826702                  /usr/lib64/libgssapi_krb5.so.2.2\n"
"7f4a1c079000-7f4a1c07b000 rw-p 0004a000 fd:00 251826702                  /usr/lib64/libgssapi_krb5.so.2.2\n"
"7f4a1c07b000-7f4a1c0de000 r-xp 00000000 fd:00 251826728                  /usr/lib64/libssl.so.1.0.1e\n"
"7f4a1c0de000-7f4a1c2dd000 ---p 00063000 fd:00 251826728                  /usr/lib64/libssl.so.1.0.1e\n"
"7f4a1c2dd000-7f4a1c2e1000 r--p 00062000 fd:00 251826728                  /usr/lib64/libssl.so.1.0.1e\n"
"7f4a1c2e1000-7f4a1c2e8000 rw-p 00066000 fd:00 251826728                  /usr/lib64/libssl.so.1.0.1e\n"
"7f4a1c2e8000-7f4a1c4a5000 r-xp 00000000 fd:00 251826726                  /usr/lib64/libcrypto.so.1.0.1e\n"
"7f4a1c4a5000-7f4a1c6a5000 ---p 001bd000 fd:00 251826726                  /usr/lib64/libcrypto.so.1.0.1e\n"
"7f4a1c6a5000-7f4a1c6bf000 r--p 001bd000 fd:00 251826726                  /usr/lib64/libcrypto.so.1.0.1e\n"
"7f4a1c6bf000-7f4a1c6cb000 rw-p 001d7000 fd:00 251826726                  /usr/lib64/libcrypto.so.1.0.1e\n"
"7f4a1c6cb000-7f4a1c6cf000 rw-p 00000000 00:00 0 \n"
"7f4a1c6cf000-7f4a1c6d6000 r-xp 00000000 fd:00 251707922                  /usr/lib64/libffi.so.6.0.1\n"
"7f4a1c6d6000-7f4a1c8d5000 ---p 00007000 fd:00 251707922                  /usr/lib64/libffi.so.6.0.1\n"
"7f4a1c8d5000-7f4a1c8d6000 r--p 00006000 fd:00 251707922                  /usr/lib64/libffi.so.6.0.1\n"
"7f4a1c8d6000-7f4a1c8d7000 rw-p 00007000 fd:00 251707922                  /usr/lib64/libffi.so.6.0.1\n"
"7f4a1c8d7000-7f4a1c94d000 r-xp 00000000 fd:00 251667007                  /usr/lib64/libgmp.so.10.2.0\n"
"7f4a1c94d000-7f4a1cb4c000 ---p 00076000 fd:00 251667007                  /usr/lib64/libgmp.so.10.2.0\n"
"7f4a1cb4c000-7f4a1cb4d000 r--p 00075000 fd:00 251667007                  /usr/lib64/libgmp.so.10.2.0\n"
"7f4a1cb4d000-7f4a1cb4e000 rw-p 00076000 fd:00 251667007                  /usr/lib64/libgmp.so.10.2.0\n"
"7f4a1cb4e000-7f4a1cb74000 r-xp 00000000 fd:00 251799555                  /usr/lib64/libhogweed.so.2.5\n"
"7f4a1cb74000-7f4a1cd73000 ---p 00026000 fd:00 251799555                  /usr/lib64/libhogweed.so.2.5\n"
"7f4a1cd73000-7f4a1cd74000 r--p 00025000 fd:00 251799555                  /usr/lib64/libhogweed.so.2.5\n"
"7f4a1cd74000-7f4a1cd75000 rw-p 00026000 fd:00 251799555                  /usr/lib64/libhogweed.so.2.5\n"
"7f4a1cd75000-7f4a1cda4000 r-xp 00000000 fd:00 251799557                  /usr/lib64/libnettle.so.4.7\n"
"7f4a1cda4000-7f4a1cfa4000 ---p 0002f000 fd:00 251799557                  /usr/lib64/libnettle.so.4.7\n"
"7f4a1cfa4000-7f4a1cfa5000 r--p 0002f000 fd:00 251799557                  /usr/lib64/libnettle.so.4.7\n"
"7f4a1cfa5000-7f4a1cfa6000 rw-p 00030000 fd:00 251799557                  /usr/lib64/libnettle.so.4.7\n"
"7f4a1cfa6000-7f4a1cfb8000 r-xp 00000000 fd:00 251770388                  /usr/lib64/libtasn1.so.6.2.3\n"
"7f4a1cfb8000-7f4a1d1b8000 ---p 00012000 fd:00 251770388                  /usr/lib64/libtasn1.so.6.2.3\n"
"7f4a1d1b8000-7f4a1d1b9000 r--p 00012000 fd:00 251770388                  /usr/lib64/libtasn1.so.6.2.3\n"
"7f4a1d1b9000-7f4a1d1ba000 rw-p 00013000 fd:00 251770388                  /usr/lib64/libtasn1.so.6.2.3\n"
"7f4a1d1ba000-7f4a1d228000 r-xp 00000000 fd:00 251976203                  /usr/lib64/libtspi.so.1.2.0\n"
"7f4a1d228000-7f4a1d427000 ---p 0006e000 fd:00 251976203                  /usr/lib64/libtspi.so.1.2.0\n"
"7f4a1d427000-7f4a1d428000 r--p 0006d000 fd:00 251976203                  /usr/lib64/libtspi.so.1.2.0\n"
"7f4a1d428000-7f4a1d42a000 rw-p 0006e000 fd:00 251976203                  /usr/lib64/libtspi.so.1.2.0\n"
"7f4a1d42a000-7f4a1d42b000 rw-p 00000000 00:00 0 \n"
"7f4a1d42b000-7f4a1d467000 r-xp 00000000 fd:00 251727400                  /usr/lib64/libp11-kit.so.0.0.0\n"
"7f4a1d467000-7f4a1d666000 ---p 0003c000 fd:00 251727400                  /usr/lib64/libp11-kit.so.0.0.0\n"
"7f4a1d666000-7f4a1d66f000 r--p 0003b000 fd:00 251727400                  /usr/lib64/libp11-kit.so.0.0.0\n"
"7f4a1d66f000-7f4a1d671000 rw-p 00044000 fd:00 251727400                  /usr/lib64/libp11-kit.so.0.0.0\n"
"7f4a1d671000-7f4a1d6d1000 r-xp 00000000 fd:00 251663921                  /usr/lib64/libpcre.so.1.2.0\n"
"7f4a1d6d1000-7f4a1d8d0000 ---p 00060000 fd:00 251663921                  /usr/lib64/libpcre.so.1.2.0\n"
"7f4a1d8d0000-7f4a1d8d1000 r--p 0005f000 fd:00 251663921                  /usr/lib64/libpcre.so.1.2.0\n"
"7f4a1d8d1000-7f4a1d8d2000 rw-p 00060000 fd:00 251663921                  /usr/lib64/libpcre.so.1.2.0\n"
"7f4a1d8d2000-7f4a1d8f8000 r-xp 00000000 fd:00 251663902                  /usr/lib64/libncurses.so.5.9\n"
"7f4a1d8f8000-7f4a1daf7000 ---p 00026000 fd:00 251663902                  /usr/lib64/libncurses.so.5.9\n"
"7f4a1daf7000-7f4a1daf8000 r--p 00025000 fd:00 251663902                  /usr/lib64/libncurses.so.5.9\n"
"7f4a1daf8000-7f4a1daf9000 rw-p 00026000 fd:00 251663902                  /usr/lib64/libncurses.so.5.9\n"
"7f4a1daf9000-7f4a1db1d000 r-xp 00000000 fd:00 251663918                  /usr/lib64/liblzma.so.5.0.99\n"
"7f4a1db1d000-7f4a1dd1c000 ---p 00024000 fd:00 251663918                  /usr/lib64/liblzma.so.5.0.99\n"
"7f4a1dd1c000-7f4a1dd1d000 r--p 00023000 fd:00 251663918                  /usr/lib64/liblzma.so.5.0.99\n"
"7f4a1dd1d000-7f4a1dd1e000 rw-p 00024000 fd:00 251663918                  /usr/lib64/liblzma.so.5.0.99\n"
"7f4a1dd1e000-7f4a1dd33000 r-xp 00000000 fd:00 251663932                  /usr/lib64/libz.so.1.2.7\n"
"7f4a1dd33000-7f4a1df32000 ---p 00015000 fd:00 251663932                  /usr/lib64/libz.so.1.2.7\n"
"7f4a1df32000-7f4a1df33000 r--p 00014000 fd:00 251663932                  /usr/lib64/libz.so.1.2.7\n"
"7f4a1df33000-7f4a1df34000 rw-p 00015000 fd:00 251663932                  /usr/lib64/libz.so.1.2.7\n"
"7f4a1df34000-7f4a1df37000 r-xp 00000000 fd:00 251660397                  /usr/lib64/libdl-2.17.so\n"
"7f4a1df37000-7f4a1e136000 ---p 00003000 fd:00 251660397                  /usr/lib64/libdl-2.17.so\n"
"7f4a1e136000-7f4a1e137000 r--p 00002000 fd:00 251660397                  /usr/lib64/libdl-2.17.so\n"
"7f4a1e137000-7f4a1e138000 rw-p 00003000 fd:00 251660397                  /usr/lib64/libdl-2.17.so\n"
"7f4a1e138000-7f4a1e2ee000 r-xp 00000000 fd:00 251660391                  /usr/lib64/libc-2.17.so\n"
"7f4a1e2ee000-7f4a1e4ee000 ---p 001b6000 fd:00 251660391                  /usr/lib64/libc-2.17.so\n"
"7f4a1e4ee000-7f4a1e4f2000 r--p 001b6000 fd:00 251660391                  /usr/lib64/libc-2.17.so\n"
"7f4a1e4f2000-7f4a1e4f4000 rw-p 001ba000 fd:00 251660391                  /usr/lib64/libc-2.17.so\n"
"7f4a1e4f4000-7f4a1e4f9000 rw-p 00000000 00:00 0 \n"
"7f4a1e4f9000-7f4a1e5fa000 r-xp 00000000 fd:00 251660399                  /usr/lib64/libm-2.17.so\n"
"7f4a1e5fa000-7f4a1e7f9000 ---p 00101000 fd:00 251660399                  /usr/lib64/libm-2.17.so\n"
"7f4a1e7f9000-7f4a1e7fa000 r--p 00100000 fd:00 251660399                  /usr/lib64/libm-2.17.so\n"
"7f4a1e7fa000-7f4a1e7fb000 rw-p 00101000 fd:00 251660399                  /usr/lib64/libm-2.17.so\n"
"7f4a1e7fb000-7f4a1e811000 r-xp 00000000 fd:00 251663873                  /usr/lib64/libpthread-2.17.so\n"
"7f4a1e811000-7f4a1ea11000 ---p 00016000 fd:00 251663873                  /usr/lib64/libpthread-2.17.so\n"
"7f4a1ea11000-7f4a1ea12000 r--p 00016000 fd:00 251663873                  /usr/lib64/libpthread-2.17.so\n"
"7f4a1ea12000-7f4a1ea13000 rw-p 00017000 fd:00 251663873                  /usr/lib64/libpthread-2.17.so\n"
"7f4a1ea13000-7f4a1ea17000 rw-p 00000000 00:00 0 \n"
"7f4a1ea17000-7f4a1eb40000 r-xp 00000000 fd:00 251976207                  /usr/lib64/libgnutls.so.28.41.0\n"
"7f4a1eb40000-7f4a1ed3f000 ---p 00129000 fd:00 251976207                  /usr/lib64/libgnutls.so.28.41.0\n"
"7f4a1ed3f000-7f4a1ed49000 r--p 00128000 fd:00 251976207                  /usr/lib64/libgnutls.so.28.41.0\n"
"7f4a1ed49000-7f4a1ed4c000 rw-p 00132000 fd:00 251976207                  /usr/lib64/libgnutls.so.28.41.0\n"
"7f4a1ed4c000-7f4a1ed6d000 r-xp 00000000 fd:00 251663920                  /usr/lib64/libselinux.so.1\n"
"7f4a1ed6d000-7f4a1ef6d000 ---p 00021000 fd:00 251663920                  /usr/lib64/libselinux.so.1\n"
"7f4a1ef6d000-7f4a1ef6e000 r--p 00021000 fd:00 251663920                  /usr/lib64/libselinux.so.1\n"
"7f4a1ef6e000-7f4a1ef6f000 rw-p 00022000 fd:00 251663920                  /usr/lib64/libselinux.so.1\n"
"7f4a1ef6f000-7f4a1ef71000 rw-p 00000000 00:00 0 \n"
"7f4a1ef71000-7f4a1ef96000 r-xp 00000000 fd:00 251663912                  /usr/lib64/libtinfo.so.5.9\n"
"7f4a1ef96000-7f4a1f196000 ---p 00025000 fd:00 251663912                  /usr/lib64/libtinfo.so.5.9\n"
"7f4a1f196000-7f4a1f19a000 r--p 00025000 fd:00 251663912                  /usr/lib64/libtinfo.so.5.9\n"
"7f4a1f19a000-7f4a1f19b000 rw-p 00029000 fd:00 251663912                  /usr/lib64/libtinfo.so.5.9\n"
"7f4a1f19b000-7f4a1f1a0000 r-xp 00000000 fd:00 252339264                  /usr/lib64/libgpm.so.2.1.0\n"
"7f4a1f1a0000-7f4a1f3a0000 ---p 00005000 fd:00 252339264                  /usr/lib64/libgpm.so.2.1.0\n"
"7f4a1f3a0000-7f4a1f3a1000 r--p 00005000 fd:00 252339264                  /usr/lib64/libgpm.so.2.1.0\n"
"7f4a1f3a1000-7f4a1f3a2000 rw-p 00006000 fd:00 252339264                  /usr/lib64/libgpm.so.2.1.0\n"
"7f4a1f3a2000-7f4a1f500000 r-xp 00000000 fd:00 252555431                  /usr/lib64/libxml2.so.2.9.1\n"
"7f4a1f500000-7f4a1f6ff000 ---p 0015e000 fd:00 252555431                  /usr/lib64/libxml2.so.2.9.1\n"
"7f4a1f6ff000-7f4a1f707000 r--p 0015d000 fd:00 252555431                  /usr/lib64/libxml2.so.2.9.1\n"
"7f4a1f707000-7f4a1f709000 rw-p 00165000 fd:00 252555431                  /usr/lib64/libxml2.so.2.9.1\n"
"7f4a1f709000-7f4a1f70b000 rw-p 00000000 00:00 0 \n"
"7f4a1f70b000-7f4a1f751000 r-xp 00000000 fd:00 251666993                  /usr/lib64/libdbus-1.so.3.7.4\n"
"7f4a1f751000-7f4a1f951000 ---p 00046000 fd:00 251666993                  /usr/lib64/libdbus-1.so.3.7.4\n"
"7f4a1f951000-7f4a1f952000 r--p 00046000 fd:00 251666993                  /usr/lib64/libdbus-1.so.3.7.4\n"
"7f4a1f952000-7f4a1f953000 rw-p 00047000 fd:00 251666993                  /usr/lib64/libdbus-1.so.3.7.4\n"
"7f4a1f953000-7f4a1f95a000 r-xp 00000000 fd:00 251663877                  /usr/lib64/librt-2.17.so\n"
"7f4a1f95a000-7f4a1fb59000 ---p 00007000 fd:00 251663877                  /usr/lib64/librt-2.17.so\n"
"7f4a1fb59000-7f4a1fb5a000 r--p 00006000 fd:00 251663877                  /usr/lib64/librt-2.17.so\n"
"7f4a1fb5a000-7f4a1fb5b000 rw-p 00007000 fd:00 251663877                  /usr/lib64/librt-2.17.so\n"
"7f4a1fb5b000-7f4a1fc3c000 r-xp 00000000 fd:00 251895343                  /usr/lib64/libasound.so.2.0.0\n"
"7f4a1fc3c000-7f4a1fe3c000 ---p 000e1000 fd:00 251895343                  /usr/lib64/libasound.so.2.0.0\n"
"7f4a1fe3c000-7f4a1fe42000 r--p 000e1000 fd:00 251895343                  /usr/lib64/libasound.so.2.0.0\n"
"7f4a1fe42000-7f4a1fe44000 rw-p 000e7000 fd:00 251895343                  /usr/lib64/libasound.so.2.0.0\n"
"7f4a1fe44000-7f4a1fe65000 r-xp 00000000 fd:00 251660384                  /usr/lib64/ld-2.17.so\n"
"7f4a20045000-7f4a20056000 rw-p 00000000 00:00 0 \n"
"7f4a20064000-7f4a20065000 rw-p 00000000 00:00 0 \n"
"7f4a20065000-7f4a20066000 r--p 00021000 fd:00 251660384                  /usr/lib64/ld-2.17.so\n"
"7f4a20066000-7f4a20067000 rw-p 00022000 fd:00 251660384                  /usr/lib64/ld-2.17.so\n"
"7f4a20067000-7f4a20068000 rw-p 00000000 00:00 0 \n"
"7fff3ce01000-7fff3ce2d000 rw-p 00000000 00:00 0                          [stack]\n"
"7fff3cf98000-7fff3cf9a000 r-xp 00000000 00:00 0                          [vdso]\n"
"ffffffffff600000-ffffffffff601000 r-xp 00000000 00:00 0                  [vsyscall]\n";

class MPIOMPTTest: public :: testing :: Test
{
    protected:
        void SetUp();
        void TearDown();
        std::string m_maps_path;
};

void MPIOMPTTest::SetUp()
{
    m_maps_path = "OMPTest.maps";
    std::ofstream maps_file(m_maps_path);
    maps_file << g_test_map;
    maps_file.close();
}

void MPIOMPTTest::TearDown()
{
    unlink(m_maps_path.c_str());
}

TEST_F(MPIOMPTTest, hello)
{
    geopm::OMPT ompt_o(m_maps_path);
    std::string name;
    ompt_o.region_name((void*)0x7f4a1b0e7000ULL, name);
    ASSERT_EQ("/usr/lib64/libresolv-2.17.so-0x0000000000000000", name);
    ompt_o.region_name((void*)0x7f4a1b0e7256ULL, name);
    ASSERT_EQ("/usr/lib64/libresolv-2.17.so-0x0000000000000256", name);
}
