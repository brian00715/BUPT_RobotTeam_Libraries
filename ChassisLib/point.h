#ifndef __point_H
#define __point_H
#ifdef __cplusplus
extern "C"
{
#endif

#define POINTS_NUM 400
  typedef struct point
  {
    float x;
    float y;
    float speed;
    float direct;
    float target_angle;
  } PlanPoint;

  extern PlanPoint points_pos0[];
  // extern PlanPoint points_pos0[POINTS_NUM];
  extern PlanPoint points_pos1[POINTS_NUM];
  extern PlanPoint points_pos2[POINTS_NUM];
  extern PlanPoint points_pos3[POINTS_NUM];
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