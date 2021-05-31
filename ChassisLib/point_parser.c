/*******************************************************************************
Copyright:      Bupt
File name:      point_parser.c
Description:    轨迹添加函数
Author:         ZX
Version：       1.0
Data:           2019/11/24
*******************************************************************************/
#include <string.h>
#include "point.h"
#include "point_parser.h"
#include "simplelib.h"

int trackTotalNum = 12;

PointContainer pContainer[] = {
    TrackAdd(points_pos1, POINTS_NUM, "CATCH_1", 1),
    TrackAdd(points_pos2, POINTS_NUM, "TOUCHDOWN_1", 2),
    TrackAdd(points_pos3, POINTS_NUM, "KICK_1", 3),
    TrackAdd(points_pos4, POINTS_NUM, "CATCH_2", 4),
    TrackAdd(points_pos5, POINTS_NUM, "TOUCHDOWN_2", 5),
    TrackAdd(points_pos6, POINTS_NUM, "TOUCHDOWN_3", 6),
    TrackAdd(points_pos7, POINTS_NUM, "TOUCHDOWN_4", 7),
    TrackAdd(points_pos8, POINTS_NUM, "KICK_2", 8),
    TrackAdd(points_pos9, POINTS_NUM, "KICK_3", 9),
    TrackAdd(points_pos10, POINTS_NUM, "KICK_4", 10),
    TrackAdd(points_pos11, POINTS_NUM, "CATCH_5", 11),
    TrackAdd(points_pos12, POINTS_NUM, "KICK_5", 12)};

PointContainer *GetTargetTarck(char tName[])
{
    int length = sizeof(pContainer) / sizeof(pContainer[0]);
    for (int i = 0; i < length; i++)
    {
        if (strcmp(pContainer[i].name, tName) == 0)
        {
            return &pContainer[i];
        }
    }
    uprintf("No track found!\r\n");
    return NULL;
}

//TODO :待测试.目前有问题,会越界
/**对路径按rank进行排序打印*/
int point_print_path()
{
    int length = sizeof(pContainer) / sizeof(pContainer[0]);
    PointContainer temp;
    for (int j = 0; j < length; j++)
    {
        for (int k = j + 1; k <= length; k++)
        {
            if (pContainer[j].rank < pContainer[k].rank)
            {
                temp = pContainer[j];
                pContainer[j] = pContainer[k];
                pContainer[k] = temp;
            }
        }
    }
    for (int i = 0; i < length; i++)
    {
        uprintf("%d---%s\r\n", pContainer[i].rank, pContainer[i].name);
    }
    return length;
}
