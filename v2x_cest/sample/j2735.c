/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include "j2735.h"


void print_hex(char *data, int len){
    printf("HEX[%d] : ",len);
    for(int i = 0 ; i < len ; i++){
        printf("%02X",(data[i] & 0xFF));
    }
    printf("\n");
}


int encode_j2735_uper(char *dst, unsigned short dstLen, MessageFrame_t *src)
{
    int res = -1;

    asn_enc_rval_t ret = uper_encode_to_buffer(&asn_DEF_MessageFrame,
                                               NULL,
                                               src,
                                               dst, dstLen);
      
    if (ret.encoded > 0)
        return ret.encoded; //  UPER Encoding Success
    else
    { 
        if (ret.failed_type != NULL)
            printf("encoded error value name = %s\n", ret.failed_type->name);

        return -1; // UPER Encoding failed
    }
}

int decode_j2735_uper(MessageFrame_t *dst, char *src, int size){ 
  
    int res = -1;

    MessageFrame_t *ptrMsg = NULL; 

    asn_dec_rval_t ret = uper_decode(NULL,
                                     &asn_DEF_MessageFrame,
                                     (void **)&dst,
                                     src, size, 0, 0);

    if (ret.code != RC_OK)
        return res;
    
    res = ret.consumed;
 
    if (dst->messageId == DSRC_ID_SPAT)
    {
        // asn_fprint(stdout,&asn_DEF_MessageFrame,dst);

    }

    // parse_decoded_j2735(dst);

    return res;
}
 
int parse_decoded_j2735(MessageFrame_t *msg)
{ 
    switch(msg->messageId)
    {
        case DSRC_ID_BSM:
            // printf(">> Parse J2735 : BSM\n");
            break;
        case DSRC_ID_SPAT:
            printf("\n\n>> Parse J2735 : SPAT\n");
            parse_spat(&msg->value.choice.SPAT);

            break;  
        case DSRC_ID_MAP:
            // printf(">> Parse J2735 : MAP\n");
            break;
    }
    return 0;
}

int parse_map(MapData_t *map){

    for (int i = 0; i < map->intersections->list.count; i++)
    {  
         struct IntersectionGeometry *ptr= map->intersections->list.array[i]; 
        // MISSION : MAP 메시지에 포함된 IntersectionGeometry별 Intersection ID 추출
        //           Intersection 내 Ref Position을 기준으로 LaneSet의 Node 좌표 계산 및 출력
        //           Node로 연결된 차선의 Line 별 ID와 SignalGroupID를 출력 
    }

    return 0;
}

int parse_spat(SPAT_t *spat)
{

    for (int i = 0; i < spat->intersections.list.count; i++)
    {
        struct IntersectionState *ptr = spat->intersections.list.array[i];

        // MISSION : MAP 메시지에 포함된 Intersection ID 추출
        //           Intersection 내 Ref Position을 기준으로 Offset Node 좌표 추출
        //           Node로 연결된 차선의 Line 별 ID와 SignalGroupID를 출력
        printf(" name        : %s\n", ptr->name->buf);      // char*  
        printf(" id.region   : %ld\n", *ptr->id.region);    // long
        printf(" id.id       : %ld\n", ptr->id.id);         // long
        printf(" revision    : %ld\n", ptr->revision);      // long
        printf(" status      : %#x\n", *ptr->status.buf);   // uint8
        printf(" moy         : %ld\n", *ptr->moy);          // long
        printf(" timestamp   : %ld\n", *ptr->timeStamp);    // long

        for (int j = 0; j < ptr->states.list.count; j++)
        {
            // struct 
            struct MovementState *state = ptr->states.list.array[j];
            printf(" -- movementName      : %s\n", state->movementName->buf);      // char*
            printf(" -- signalGroup       : %ld\n", state->signalGroup);           // long
            
            for (int k = 0; k < state->state_time_speed.list.count; k++)
            {
                printf(" ---- eventState        : %ld\n", state->state_time_speed.list.array[k]->eventState);                    // long
                printf(" ---- timing_minEndTime : %ld\n", state->state_time_speed.list.array[k]->timing->minEndTime);     // long
            }

            for (int l = 0; l < state->maneuverAssistList->list.count; l++)
            {
                printf(" ------ connectionID      : %ld\n", state->maneuverAssistList->list.array[l]->connectionID);      // char*
                printf(" ------ pedBicycleDetect  : %s\n", *state->maneuverAssistList->list.array[l]->pedBicycleDetect ? "TRUE" : "FALSE");      // char*
            }

        }

    }

    return 0;
}
int parse_bsm(BasicSafetyMessage_t *bsm){
 
    // MISSION : BSM 내 temporary ID 추출
    //           차량의 위치(위도,경도,고도)와 주행 방향, 속도 출력
 
    return 0;
}