#include "fuzzy.h"

/* ===== CRISP OUTPUT VALUES ===== */
#define OUT_RVS  (-80.0f)
#define OUT_MID   (65.0f)
#define OUT_FAST  (85.0f)
#define OUT_MAX  (100.0f)

/* ===== LAYER 1: MEMBERSHIP FUNCTIONS =====
   NEAR : full [0-15],  zero t?i 25
   MID  : rise [15-40], fall [40-65]   ? min-sensor ch? xét trong vùng này
   FAR  : rise [55-65], full t?i 65+
*/
static float mu_near(float x) {
    if(x <= 15.0f) return 1.0f;
    if(x >= 25.0f) return 0.0f;
    return (25.0f - x) / 10.0f;
}
static float mu_mid(float x) {
    if(x <= 15.0f || x >= 65.0f) return 0.0f;
    if(x <= 40.0f) return (x - 15.0f) / 25.0f;
    return (65.0f - x) / 25.0f;
}
static float mu_far(float x) {
    if(x <= 55.0f) return 0.0f;
    if(x >= 65.0f) return 1.0f;
    return (x - 55.0f) / 10.0f;
}
/* M?c d? a < b (dùng cho di?u ki?n min-sensor) */
static float fuzzy_less(float a, float b) {
    float d = b - a;
    if(d >= 20.0f) return 1.0f;
    if(d <=  0.0f) return 0.0f;
    return d / 20.0f;
}

void Fuzzy_Control(float l, float m, float r, int16_t *ls, int16_t *rs)
{
    /* L?c giá tr? l?i t? thu vi?n sonar */
    if(l <= 0.1f) l = 999.0f;
    if(m <= 0.1f) m = 999.0f;
    if(r <= 0.1f) r = 999.0f;

    /* ===== LAYER 1: TÍNH Ð? THU?C ===== */
    float nL = mu_near(l), mL = mu_mid(l), fL = mu_far(l);
    float nM = mu_near(m), mM = mu_mid(m), fM = mu_far(m);
    float nR = mu_near(r), mR = mu_mid(r), fR = mu_far(r);

    /* ===== LAYER 2: TR?NG S? T?NG RULE =====
       R1: M=NEAR           ? húc th?ng
       R2: L=NEAR           ? quay trái húc
       R3: R=NEAR           ? quay ph?i húc
       R4: min=M, MID range ? ti?n th?ng nhanh
       R5: min=L, MID range ? xoay trái nh? húc
       R6: min=R, MID range ? xoay ph?i nh? húc
       R7: all=FAR          ? search
    */
    float w1 = nM;
    float w2 = nL;
    float w3 = nR;
    float w4 = mM * (1.0f - nL) * (1.0f - nR)
             * fuzzy_less(m, l) * fuzzy_less(m, r);
    float w5 = mL * (1.0f - nM) * (1.0f - nR)
             * fuzzy_less(l, m) * fuzzy_less(l, r);
    float w6 = mR * (1.0f - nL) * (1.0f - nM)
             * fuzzy_less(r, l) * fuzzy_less(r, m);
    float w7 = fL * fM * fR;

    /* ===== LAYER 3: NORMALIZE ===== */
    float w_sum = w1 + w2 + w3 + w4 + w5 + w6 + w7;
    if(w_sum < 1e-6f) {          // Không rule nào fire ? default search
        *ls =  (int16_t)OUT_FAST;
        *rs = -(int16_t)OUT_FAST;
        return;
    }
    float wn1=w1/w_sum, wn2=w2/w_sum, wn3=w3/w_sum;
    float wn4=w4/w_sum, wn5=w5/w_sum, wn6=w6/w_sum, wn7=w7/w_sum;

    /* ===== LAYER 4: CRISP OUTPUT T?NG RULE ===== */
    //         R1        R2        R3        R4        R5        R6        R7
    float LS[] = {OUT_MAX, OUT_RVS, OUT_MAX, OUT_FAST, OUT_MID,  OUT_FAST, OUT_FAST};
    float RS[] = {OUT_MAX, OUT_MAX, OUT_RVS, OUT_FAST, OUT_FAST, OUT_MID,  OUT_RVS };
    float wn[]  = {wn1,     wn2,     wn3,     wn4,      wn5,      wn6,      wn7    };

    /* ===== LAYER 5: WEIGHTED AVERAGE ===== */
    float ls_f = 0.0f, rs_f = 0.0f;
    for(int i = 0; i < 7; i++) {
        ls_f += wn[i] * LS[i];
        rs_f += wn[i] * RS[i];
    }

    /* Clamp -100 ~ 100 */
    if(ls_f >  100.0f) ls_f =  100.0f;
    if(ls_f < -100.0f) ls_f = -100.0f;
    if(rs_f >  100.0f) rs_f =  100.0f;
    if(rs_f < -100.0f) rs_f = -100.0f;

    *ls = (int16_t)ls_f;
    *rs = (int16_t)rs_f;
}