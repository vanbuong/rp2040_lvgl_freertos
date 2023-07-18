/**
 * \file            lwesp_opts.h
 * \brief           ESP application options
 */

/*
 * Copyright (c) 2023 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwESP - Lightweight ESP-AT parser library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         v1.1.0-dev
 */
#ifndef LWESP_HDR_OPTS_H
#define LWESP_HDR_OPTS_H

/* Rename this file to "lwesp_opts.h" for your application */

/*
 * Open "include/lwesp/lwesp_opt.h" and
 * copy & replace here settings you want to change values
 */

#define LWESP_CFG_DBG_INIT                    LWESP_DBG_ON
#define LWESP_CFG_DBG                         LWESP_DBG_ON
#define LWESP_CFG_DBG_TYPES_ON                1
#define LWESP_CFG_AT_ECHO                     1
#define LWESP_CFG_INPUT_USE_PROCESS           1

#define LWESP_CFG_NETCONN                     1
#define LWESP_CFG_ESP32_C3                    1
#define LWESP_CFG_ESP8266                     0
#define LWESP_CFG_ESP32                       0
#define LWESP_CFG_MODE_ACCESS_POINT           0
#define LWESP_CFG_KEEP_ALIVE                  0
#define LWESP_CFG_RESET_ON_INIT               1
#define LWESP_CFG_RESTORE_ON_INIT             0
#define LWESP_CFG_RESET_ON_DEVICE_PRESENT     0

#endif /* LWESP_HDR_OPTS_H */
