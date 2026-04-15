#include "fuzzy.h"
#include <math.h>

void Fuzzy_Control(float l, float m, float r, int16_t *ls, int16_t *rs)
{
    /* ===== 1. L?C NHI?U VÀ S?A L?I THU VI?N SONAR ===== */
    // Thu vi?n cu tr? v? <= 0 (ho?c -1) khi quá timeout ho?c không th?y v?t.
    // B?t bu?c ph?i ép các giá tr? l?i này v? vô c?c (999) d? xe không hi?u nh?m là v?t ? sát m?t.
    if (l <= 0.1f) l = 999.0f;
    if (m <= 0.1f) m = 999.0f;
    if (r <= 0.1f) r = 999.0f;

    /* ===== 2. TÌM KI?M (SEARCH) ===== */
    // Không có v?t c?n nào trong bán kính 80cm
    if (m > 80.0f && l > 80.0f && r > 80.0f)
    {
        // Xoay tròn d? tìm ki?m v?i t?c d? d? th?ng l?c ma sát tinh (Ðã tang m?nh t? 55 lên 85)
        *ls = 85;  
        *rs = -85; 
        return; // Thoát ngay d? không ch?y các lu?ng bên du?i
    }

    /* ===== 3. LU?NG ÐI?U KI?N: KHÓA GÓC & T?N CÔNG ===== */
    if (m <= 80.0f) 
    {
        // TRU?NG H?P A: C?M BI?N GI?A ÐÃ TH?Y V?T
        if (m <= 20.0f) 
        {
            // V?t ? sát m?t (< 20cm) -> Bom max 100% công su?t d? húc d?y ra kh?i vòng
            *ls = 85;
            *rs = 85;
        } 
        else 
        {
            // Ðang lao t?i, ki?m tra xem v?t có l?ch sang 2 bên không d? n?n th?ng lái
            if (l < m && l < 80.0f) 
            {
                // V?t g?n bên trái hon -> Ðánh lái sang TRÁI (Bánh ph?i d?y m?nh 90, bánh trái chùn l?i 60)
                *ls = 60;
                *rs = 90;
            } 
            else if (r < m && r < 80.0f) 
            {
                // V?t g?n bên ph?i hon -> Ðánh lái sang PH?I (Bánh trái d?y m?nh 90, bánh ph?i chùn l?i 60)
                *ls = 90;
                *rs = 60;
            } 
            else 
            {
                // V?t n?m ngay chính gi?a -> Lao th?ng t?c d? cao
                *ls = 85;
                *rs = 85;
            }
        }
    }
    else 
    {
        // TRU?NG H?P B: GI?A KHÔNG TH?Y, NHUNG TRÁI/PH?I TH?Y (C?n khóa góc)
        if (l <= 80.0f && r > 80.0f) 
        {
            // Ch? TRÁI th?y -> Lùi bánh trái, ti?n bánh ph?i d? quay g?t d?u xe sang trái (Ðã tang l?c lên -50 và 85)
            // Ép d?i th? l?t vào t?m nhìn c?a con Sonar gi?a
            *ls = -70;
            *rs = 85;
        } 
        else if (r <= 80.0f && l > 80.0f) 
        {
            // Ch? PH?I th?y -> Lùi bánh ph?i, ti?n bánh trái d? quay g?t d?u xe sang ph?i
            *ls = 85;
            *rs = -70;
        }
        else if (l <= 80.0f && r <= 80.0f) 
        {
            // Tru?ng h?p hy h?u: 2 bên d?u th?y nhung gi?a không th?y (xe d?i th? to/d?t)
            *ls = 70;
            *rs = 70;
        }
    }

    /* ===== 4. GI?I H?N AN TOÀN PWM ===== */
    // Ð?m b?o PWM truy?n vào module di?u khi?n d?ng co luôn n?m trong ngu?ng -100 d?n 100
    if (*ls > 100) *ls = 100;
    if (*ls < -100) *ls = -100;
    
    if (*rs > 100) *rs = 100;
    if (*rs < -100) *rs = -100;
}