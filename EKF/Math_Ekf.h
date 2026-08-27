#ifndef _MATH_EKF_H
#define _MATH_EKF_H
#include <EKF.h>
void EurlToDcm(Matrix* Rotation_M, RowVector *Eurl);
Quaternion EurlToQuaternion(RowVector *Eurl);
RowVector QuaternionToEulerAngles(Quaternion q);
RowVector  matrix_vector_multiply(const Matrix* mat, const RowVector* vec);
RowVector Add_vector(const RowVector* vec_1,const RowVector* vec_2);
RowVector Sub_vector(const RowVector* vec_1,const RowVector* vec_2);
Matrix transpose( Matrix *Cnb);
RowVector cross_product(RowVector *a, RowVector *b);
Matrix _Askew(RowVector *a,int sign);
RowVector Number_multiply_vector(float number ,RowVector *a);
void Number_multiply_Matrix_3_3(float number ,Matrix *a);
void Number_multiply_Matrix_15_15(float number ,Matrix_15_15 *a);
Matrix multiply_3_3(Matrix *A, Matrix *B);
Matrix addMatrices_3_3(Matrix *A, Matrix *B);
void addMatrices_15_15(Matrix_15_15 *A, Matrix_15_15 *B,Matrix_15_15 *C);
void Identity_matrix_15_15(Matrix_15_15 *M);
float Vector_Module_2(RowVector *a);
void Add_Matrix_Updata(Matrix*M,uint8_t row,uint8_t column);
void State_Matrix_Updata(Matrix*Maa,Matrix*Mav,Matrix*Map,Matrix*Mva,Matrix*Mvv,Matrix*Mvp,Matrix*Mpv,Matrix*Mpp);
Quaternion Quaternion_Update(RowVector *W,Quaternion Q,int Direction);
RowVector_15  matrix_vector_multiply_15_15(const Matrix_15_15* mat, const RowVector_15* vec) ;
void transpose_15_15( Matrix_15_15 *M);//¾ØÕó×ªÖÃ
void multiply_15_15(Matrix_15_15 *A, Matrix_15_15 *B,Matrix_15_15 *C);
Matrix_6_6 matrixInverse(Matrix_6_6 *input);
Matrix_15_6 transpose_Matrix_6_15(Matrix_6_15* src);
Matrix_15_6 multiply_15_15_15_6(Matrix_15_15 *A, Matrix_15_6 *B);
Matrix_6_6 multiply_6_15_15_6(Matrix_6_15 *A, Matrix_15_6 *B);
Matrix_6_6 addMatrices_6_6(Matrix_6_6 *A, Matrix_6_6 *B);
Matrix_15_6 multiply_15_6_6_6(Matrix_15_6 *A, Matrix_6_6 *B);
RowVector_6  matrix_vector_multiply_6_15(const Matrix_6_15* mat, const RowVector_15* vec);
RowVector_6 Sub_vector_6(const RowVector_6* vec_1,const RowVector_6* vec_2);
RowVector_15  matrix_vector_multiply_15_6(const Matrix_15_6* mat, const RowVector_6* vec) ;
RowVector_15 Add_vector_15(const RowVector_15* vec_1,const RowVector_15* vec_2);
Matrix_15_15 multiply_15_6_6_15(Matrix_15_6 *A, Matrix_6_15 *B);
Matrix_15_15 SubMatrices_15_15(Matrix_15_15 *A, Matrix_15_15 *B);
Matrix_15_15 Number_multiply_Matrix_15_15_(float number ,Matrix_15_15 a);
Matrix_15_15 multiply_15_15_(Matrix_15_15 *A, Matrix_15_15 *B);
Matrix_15_3 transpose_Matrix_3_15(Matrix_3_15* src);
Matrix_15_3 multiply_15_15_15_3(Matrix_15_15 *A, Matrix_15_3 *B);
Matrix multiply_3_15_15_3(Matrix_3_15 *A, Matrix_15_3 *B);
Matrix matrixInverse_3_3(Matrix *input );
Matrix_15_3 multiply_15_3_3_3(Matrix_15_3 *A, Matrix *B);
RowVector  matrix_vector_multiply_3_15(const Matrix_3_15* mat, const RowVector_15* vec) ;
RowVector_15  matrix_vector_multiply_15_3(const Matrix_15_3* mat, const RowVector* vec);
Matrix_15_15 multiply_15_3_3_15(Matrix_15_3 *A, Matrix_3_15 *B);











#endif