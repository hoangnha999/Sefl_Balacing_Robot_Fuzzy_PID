#include <Arduino.h>

#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);


// Số lượng đầu vào của hệ thống suy luận mờ
const int fis_gcI = 1;
// Số lượng đầu ra của hệ thống suy luận mờ
const int fis_gcO = 3;
// Số lượng luật của hệ thống suy luận mờ
const int fis_gcR = 7;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];
void fis_evaluate();


//***********************************************************************
// Các hàm hỗ trợ cho hệ thống suy luận mờ                          
//***********************************************************************
// Hàm thành viên hình tam giác
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
    return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
    return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Dữ liệu cho hệ thống suy luận mờ                                       
//***********************************************************************
// Con trỏ đến các hàm thành viên
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Số lượng hàm thành viên cho mỗi đầu vào
int fis_gIMFCount[] = { 7 };

// Số lượng hàm thành viên cho mỗi đầu ra 
int fis_gOMFCount[] = { 7, 7, 7 };

// Các hệ số cho hàm thành viên đầu vào
FIS_TYPE fis_gMFI0Coeff1[] = { -0.1667, 0, 0.1667 };
FIS_TYPE fis_gMFI0Coeff2[] = { 0, 0.1667, 0.3333 };
FIS_TYPE fis_gMFI0Coeff3[] = { 0.1667, 0.3333, 0.5 };
FIS_TYPE fis_gMFI0Coeff4[] = { 0.3333, 0.5, 0.6667 };
FIS_TYPE fis_gMFI0Coeff5[] = { 0.5, 0.6667, 0.8333 };
FIS_TYPE fis_gMFI0Coeff6[] = { 0.6667, 0.8333, 1 };
FIS_TYPE fis_gMFI0Coeff7[] = { 0.8333, 1, 1.167 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5, fis_gMFI0Coeff6, fis_gMFI0Coeff7 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff };

// Các hệ số cho hàm thành viên đầu ra
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 0.166666666666667 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0, 0.333333333333333 };
FIS_TYPE fis_gMFO0Coeff4[] = { 0, 0.5 };
FIS_TYPE fis_gMFO0Coeff5[] = { 0, 0.666666666666667 };
FIS_TYPE fis_gMFO0Coeff6[] = { 0, 0.833333333333333 };
FIS_TYPE fis_gMFO0Coeff7[] = { 0, 1 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5, fis_gMFO0Coeff6, fis_gMFO0Coeff7 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0 };
FIS_TYPE fis_gMFO1Coeff2[] = { 0, 0.166666666666667 };
FIS_TYPE fis_gMFO1Coeff3[] = { 0, 0.333333333333333 };
FIS_TYPE fis_gMFO1Coeff4[] = { 0, 0.5 };
FIS_TYPE fis_gMFO1Coeff5[] = { 0, 0.666666666666667 };
FIS_TYPE fis_gMFO1Coeff6[] = { 0, 0.833333333333333 };
FIS_TYPE fis_gMFO1Coeff7[] = { 0, 1 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3, fis_gMFO1Coeff4, fis_gMFO1Coeff5, fis_gMFO1Coeff6, fis_gMFO1Coeff7 };
FIS_TYPE fis_gMFO2Coeff1[] = { 0, 0 };
FIS_TYPE fis_gMFO2Coeff2[] = { 0, 0.166666666666667 };
FIS_TYPE fis_gMFO2Coeff3[] = { 0, 0.333333333333333 };
FIS_TYPE fis_gMFO2Coeff4[] = { 0, 0.5 };
FIS_TYPE fis_gMFO2Coeff5[] = { 0, 0.666666666666667 };
FIS_TYPE fis_gMFO2Coeff6[] = { 0, 0.833333333333333 };
FIS_TYPE fis_gMFO2Coeff7[] = { 0, 1 };
FIS_TYPE* fis_gMFO2Coeff[] = { fis_gMFO2Coeff1, fis_gMFO2Coeff2, fis_gMFO2Coeff3, fis_gMFO2Coeff4, fis_gMFO2Coeff5, fis_gMFO2Coeff6, fis_gMFO2Coeff7 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff, fis_gMFO2Coeff };

// Tập hàm thành viên đầu vào
int fis_gMFI0[] = { 0, 0, 0, 0, 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0};

// Tập hàm thành viên đầu ra

int* fis_gMFO[] = {};

// Trọng số của các luật
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1 };

// Loại luật
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1 };

// Đầu vào của các luật
int fis_gRI0[] = { 1 };
int fis_gRI1[] = { 2 };
int fis_gRI2[] = { 3 };
int fis_gRI3[] = { 4 };
int fis_gRI4[] = { 5 };
int fis_gRI5[] = { 6 };
int fis_gRI6[] = { 7 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6 };

// Đầu ra của các luật
int fis_gRO0[] = { 1, 1, 1 };
int fis_gRO1[] = { 2, 2, 2 };
int fis_gRO2[] = { 3, 3, 3 };
int fis_gRO3[] = { 4, 4, 4 };
int fis_gRO4[] = { 5, 5, 5 };
int fis_gRO5[] = { 6, 6, 6 };
int fis_gRO6[] = { 7, 7, 7 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6 };

// Phạm vi đầu vào tối thiểu
FIS_TYPE fis_gIMin[] = { 0 };

// Phạm vi đầu vào tối đa
FIS_TYPE fis_gIMax[] = { 1 };

// Phạm vi đầu ra tối thiểu
FIS_TYPE fis_gOMin[] = { 0, 0, 0 };

// Phạm vi đầu ra tối đa
FIS_TYPE fis_gOMax[] = { 1, 1, 1 };

//***********************************************************************
// Các hàm hỗ trợ phụ thuộc dữ liệu cho hệ thống suy luận mờ
//***********************************************************************
// Không có cho Sugeno

//***********************************************************************
// Hệ thống suy luận mờ
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput2[] = { 0, 0, 0, 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, fuzzyOutput2, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    // FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };  // Biến này không được sử dụng
    FIS_TYPE sW = 0;

    // Chuyển đổi đầu vào thành đầu vào mờ
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = 1;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = 0;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            FIS_TYPE sWI = 0.0;
            for (j = 0; j < fis_gOMFCount[o]; ++j)
            {
                fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
                for (i = 0; i < fis_gcI; ++i)
                {
                    fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
                }
            }

            for (r = 0; r < fis_gcR; ++r)
            {
                index = fis_gRO[r][o] - 1;
                sWI += fuzzyFires[r] * fuzzyOutput[o][index];
            }

            g_fisOutput[o] = sWI / sW;
        }
    }
}
