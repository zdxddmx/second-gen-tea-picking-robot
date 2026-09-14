/**
 * @file    status.c
 * @brief   String mapping for @ref tp_status_t.
 * @copyright MIT License, see LICENSE.
 */

#include "status.h"

const char *tp_status_str(tp_status_t status)
{
    switch (status) {
    case TP_OK:               return "TP_OK";
    case TP_ERR_PARAM:        return "TP_ERR_PARAM";
    case TP_ERR_STATE:        return "TP_ERR_STATE";
    case TP_ERR_TIMEOUT:      return "TP_ERR_TIMEOUT";
    case TP_ERR_CRC:          return "TP_ERR_CRC";
    case TP_ERR_RANGE:        return "TP_ERR_RANGE";
    case TP_ERR_NO_MEM:       return "TP_ERR_NO_MEM";
    case TP_ERR_NOT_READY:    return "TP_ERR_NOT_READY";
    case TP_ERR_UNSUPPORTED:  return "TP_ERR_UNSUPPORTED";
    case TP_ERR_IO:           return "TP_ERR_IO";
    case TP_ERR_FAULT_ACTIVE: return "TP_ERR_FAULT_ACTIVE";
    case TP_ERR_BUSY:         return "TP_ERR_BUSY";
    case TP_ERR_OVERFLOW:     return "TP_ERR_OVERFLOW";
    case TP_ERR_NOT_FOUND:    return "TP_ERR_NOT_FOUND";
    case TP_ERR_PROTOCOL:     return "TP_ERR_PROTOCOL";
    case TP_ERR_DIVERGED:     return "TP_ERR_DIVERGED";
    default:                  return "TP_ERR_UNKNOWN";
    }
}

const char *tp_status_desc(tp_status_t status)
{
    switch (status) {
    case TP_OK:               return "ok";
    case TP_ERR_PARAM:        return "invalid parameter";
    case TP_ERR_STATE:        return "illegal state";
    case TP_ERR_TIMEOUT:      return "timeout";
    case TP_ERR_CRC:          return "crc mismatch";
    case TP_ERR_RANGE:        return "out of range";
    case TP_ERR_NO_MEM:       return "out of memory";
    case TP_ERR_NOT_READY:    return "not ready";
    case TP_ERR_UNSUPPORTED:  return "unsupported";
    case TP_ERR_IO:           return "io error";
    case TP_ERR_FAULT_ACTIVE: return "fault active";
    case TP_ERR_BUSY:         return "busy";
    case TP_ERR_OVERFLOW:     return "overflow";
    case TP_ERR_NOT_FOUND:    return "not found";
    case TP_ERR_PROTOCOL:     return "protocol error";
    case TP_ERR_DIVERGED:     return "numeric divergence";
    default:                  return "unknown error";
    }
}
