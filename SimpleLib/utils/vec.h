#ifndef __vec_H
#define __vec_H
#ifdef __cplusplus
extern "C"
{
#endif
    /*Struct Area*/
    typedef struct
    {
        float x;
        float y;
    } vec;
    /*Function Area*/
    vec Vec_Create(float x, float y);     // 向量生成
    vec Vec_Add(vec a, vec b);            // 向量加法
    double Vec_DotProduct(vec a, vec b); // 向量点乘
    double Vec_Model(vec a);              // 向量取模
    vec Vec_ScalarMul(vec a, double b);  // 向量数乘
    vec Vec_Normal(vec a);                // 向量顺时针90度法向
    vec Vec_Unit(vec a);                  // 向量归一化
    int Vec_IsZero(vec a);               // 向量模是否为0
    float Vec_PhaseAngleSub(vec b, vec a);
    float Vec_GetPhaseAngle(vec a);

#ifdef __cplusplus
}
#endif
#endif
