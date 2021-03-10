#ifndef __point_parser_H
#define __point_parser_H
#ifdef __cplusplus
 extern "C" {
#endif

#include"point.h"
/**
 * targetPoints: Point 数组指针
 * pointsNum : 点集数目
 * pointsName : 点集名字字符串
 * rank : 点集顺序
*/
#define TrackAdd(targetPoints, pointsNum, pointsName,rank) \
    { \
        targetPoints, \
        pointsNum, \
        pointsName, \
        rank\
    } \

typedef struct{
  PlanPoint *point_array;
  int point_num;
  char name[16];
  int rank;
}PointContainer;

extern PointContainer pContainer[];

PointContainer* GetTargetTarck(char tName[]);
int point_print_path();

#ifdef __cplusplus
}
#endif
#endif /*__point_parser_H */