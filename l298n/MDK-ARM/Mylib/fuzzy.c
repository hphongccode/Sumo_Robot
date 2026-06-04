#include "fuzzy.h"

/* -----------------------------------------------------------
   CRISP OUTPUT VALUES
   ---------------------------------------------------------
   OUT_CURVE_NEAR (35): T?c d? bánh CH?M khi húc cong – G?N
     ? Robot ti?n c? 2 bánh, l?ch t?c ? lao th?ng vào v?t c?n
     ? Thay th? ki?u xoay t?i ch? cu (1 bánh lùi) r?t ch?m
   
   OUT_CURVE_MID (55): T?c d? bánh CH?M khi húc cong – TRUNG
     ? Cong nh? hon NEAR, dùng khi v?t c?n ? kho?ng 25-55cm
   ----------------------------------------------------------- */
#define OUT_RVS         (-85.0f)
#define OUT_CURVE_NEAR  (60.0f)   /* Bánh ch?m – húc cong G?N  */
#define OUT_CURVE_MID   (70.0f)   /* Bánh ch?m – húc cong TRUNG */
#define OUT_FAST        (90.0f)
#define OUT_MAX         (100.0f)

/* ----------- MEMBERSHIP FUNCTIONS ----------- */

static float mu_near(float x) {
    if (x <= 15.0f) return 1.0f;
    if (x >= 25.0f) return 0.0f;
    return (25.0f - x) / 10.0f;
}

static float mu_mid(float x) {
    if (x <= 20.0f || x >= 55.0f) return 0.0f;
    if (x <= 30.0f) return (x - 20.0f) / 10.0f;
    if (x <= 45.0f) return 1.0f;
    return (55.0f - x) / 10.0f;
}

static float mu_far(float x) {
    if (x <= 50.0f) return 0.0f;
    if (x >= 60.0f) return 1.0f;
    return (x - 50.0f) / 10.0f;
}

/* ----------------------------------------------------------
   [ÐÃ S?A L?I ÐI?M MÙ]: fuzzy_less
   - Giúp h?p nh?t góc nhìn khi 2 c?m bi?n cùng th?y v?t c?n.
   - Tránh vi?c 2 c?m bi?n cùng kho?ng cách t? tri?t tiêu nhau.
   ---------------------------------------------------------- */
static float fuzzy_less(float a, float b) {
    float d = b - a;
    
    /* N?u b xa hon a rõ r?t (d >= 10) -> a ch?c ch?n g?n hon */
    if (d >= 10.0f) return 1.0f;
    
    /* N?u b g?n hon a rõ r?t (d <= -10) -> a không h? g?n hon */
    if (d <= -10.0f) return 0.0f;
    
    /* N?u a và b ngang ngang nhau -> chia d?u tr?ng s? d? hòa tr?n */
    return (d + 10.0f) / 20.0f;
}

/* ----------------------------------------------------------- */
void Fuzzy_Control(float l, float m, float r, int16_t *ls, int16_t *rs)
{
    if (l <= 0.1f) l = 999.0f;
    if (m <= 0.1f) m = 999.0f;
    if (r <= 0.1f) r = 999.0f;

    /* ===== LAYER 1: Ð? THU?C ===== */
    float nL = mu_near(l), mL = mu_mid(l), fL = mu_far(l);
    float nM = mu_near(m), mM = mu_mid(m), fM = mu_far(m);
    float nR = mu_near(r), mR = mu_mid(r), fR = mu_far(r);

    /* ===== LAYER 2: TR?NG S? ===== */
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

    if (w_sum < 1e-6f) {
        /* Không rule nào fire -> xoay PH?I tìm ki?m */
        *ls =  (int16_t)OUT_FAST;
        *rs = -(int16_t)OUT_FAST;
        return;
    }

    float wn1 = w1/w_sum, wn2 = w2/w_sum, wn3 = w3/w_sum;
    float wn4 = w4/w_sum, wn5 = w5/w_sum, wn6 = w6/w_sum, wn7 = w7/w_sum;

    /*            R1        R2              R3              R4        R5             R6             R7        */
    float LS[] = {OUT_MAX,  OUT_CURVE_NEAR, OUT_MAX,        OUT_FAST, OUT_CURVE_MID, OUT_FAST,      OUT_FAST };
    float RS[] = {OUT_MAX,  OUT_MAX,        OUT_CURVE_NEAR, OUT_FAST, OUT_FAST,      OUT_CURVE_MID, OUT_RVS  };
    float wn[] = {wn1,      wn2,            wn3,            wn4,      wn5,           wn6,           wn7      };

    /* ===== LAYER 5: WEIGHTED AVERAGE ===== */
    float ls_f = 0.0f, rs_f = 0.0f;
    for (int i = 0; i < 7; i++) {
        ls_f += wn[i] * LS[i];
        rs_f += wn[i] * RS[i];
    }

    if (ls_f >  100.0f) ls_f =  100.0f;
    if (ls_f < -100.0f) ls_f = -100.0f;
    if (rs_f >  100.0f) rs_f =  100.0f;
    if (rs_f < -100.0f) rs_f = -100.0f;

    *ls = (int16_t)ls_f;
    *rs = (int16_t)rs_f;
}