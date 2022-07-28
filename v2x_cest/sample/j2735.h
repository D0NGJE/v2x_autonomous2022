/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#ifndef _J2735_H_
#define _J2735_H_

#include "j2735/MessageFrame.h"
 
#include <string.h>


typedef struct PVD_MSG
{
    int year;
    int month;
    int day;
    int hour;
    int min;
    int sec;

    int pre_year;
    int pre_month;
    int pre_day;
    int pre_hour;
    int pre_min;
    int pre_sec;
    
    double lat;
    double lon;
    double elevation;
    double heading;
    double speed;
    double transmission;
    double pre_lat;
    double pre_lon;
    double pre_elevation;
    double pre_heading;
    double pre_speed;
    double pre_transmission;
}PVDMSG, *PPVDMSG;

/**
 * @brief Binary 데이터 Hex 출력 
 * 
 * @param data Binary Data 
 * @param len  Binary Data Size
 */
void print_hex(char *data, int len);

/**
 * @brief MessageFrame UPER 인코딩 
 * 
 * @param dst     UPER 인코딩된 Binary 데이터 저장 버퍼 Pointer
 * @param dstLen  저장 버퍼 크기 
 * @param src     MessageFrame Pointer
 * @return int    인코딩된 데이터 크기 
 */
int encode_j2735_uper(char *dst,unsigned short dstLen,MessageFrame_t *src);

/**
 * @brief      UPER로 인코딩된 데이터 MessageFrame으로 디코딩
 * 
 * @param dst  Dst MessageFrame Pointer
 * @param src  디코딩할 UPER 데이터 Pointer
 * @param size  UPER 데이터 길이 
 * @return int  디코딩 성공 유무
 */
int decode_j2735_uper(MessageFrame_t *dst,char *src, int size);

/**
 * @brief      디코딩된 MessageFrame 처리 
 * 
 * @param msg   MessageFrame Pointer
 * @return int  메시지 처리 성공 유무 
 */
int parse_decoded_j2735(MessageFrame_t *msg);
 

/**
 * @brief  PVD 메시지 생성 
 * 
 * @param dst  MessageFrame Pointer
 * @return int 
 */
int fill_j2735_pvd(MessageFrame_t *dst, PPVDMSG recv_buf);

/**
 * @brief  BSM 메시지 생성 
 * 
 * @param dst   MessageFrame Pointer
 * @return int  
 */
int fill_j2735_bsm(MessageFrame_t *dst);

/**
 * @brief   수신된 MAP 메시지 처리
 * 
 * @param map 
 * @return int 
 */
int parse_map(MapData_t *map);

/**
 * @brief  수신된 SPAT 메시지 처리
 * 
 * @param spat 
 * @return int 
 */
int parse_spat(SPAT_t *spat);

/**
 * @brief  수신된 BSM 메시지 처리
 * 
 * @param bsm 
 * @return int 
 */
int parse_bsm(BasicSafetyMessage_t *bsm);


enum J2735_DsrcID
{
    DSRC_ID_MAP = 18,
    DSRC_ID_SPAT = 19,
    DSRC_ID_BSM = 20 
};

#endif