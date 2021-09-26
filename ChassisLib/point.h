#ifndef __point_H
#define __point_H
#ifdef __cplusplus
extern "C"
{
#endif

  extern unsigned int points_pos1_num;
  extern unsigned int points_pos2_num;

#define POINTS_NUM 200
  typedef struct point
  {
    float x;
    float y;
    float speed;
    float direct;       // rad
    float target_angle; // rad
  } PlanPoint;

#define Points PlanPoint

  extern PlanPoint points_pos1[];
  extern PlanPoint points_pos2[];
  extern PlanPoint points_pos3[];
  extern PlanPoint points_pos4[POINTS_NUM];
  extern PlanPoint points_pos5[POINTS_NUM];
  extern PlanPoint points_pos6[POINTS_NUM];
  extern PlanPoint points_pos7[POINTS_NUM];
  extern PlanPoint points_pos8[POINTS_NUM];
  extern PlanPoint points_pos9[POINTS_NUM];
  extern PlanPoint points_pos10[POINTS_NUM];
  extern PlanPoint points_pos11[POINTS_NUM];
  extern PlanPoint points_pos12[POINTS_NUM];

#ifdef __cplusplus
}
#endif
#endif /*__ points_H */