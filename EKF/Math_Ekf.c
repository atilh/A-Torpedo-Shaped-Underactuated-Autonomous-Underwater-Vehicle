#include <imu.h>
#include <math.h>
#include <stdlib.h>
#include <Math_Ekf.h>
void EurlToDcm(Matrix* Rotation_M, RowVector *Eurl)//欧拉角转方向余弦阵
{
    float phi = Eurl->data[0];//45.0*P_I/180.0;顺序有待验证
    float theta = Eurl->data[1];//45.0 * P_I / 180.0;
    float psi = Eurl->data[2];// 90.0 * P_I / 180.0;
    Rotation_M->data[0][0] = cos(psi) * cos(theta);
    Rotation_M->data[0][1] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
    Rotation_M->data[0][2] = sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi);
    Rotation_M->data[1][0] = sin(psi) * cos(theta);
    Rotation_M->data[1][1] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
    Rotation_M->data[1][2] = -sin(phi) * cos(psi) + sin(psi) * sin(theta) * cos(phi);
    Rotation_M->data[2][0] = -sin(theta);
    Rotation_M->data[2][1] = cos(theta) * sin(phi);
    Rotation_M->data[2][2] = cos(theta) * cos(phi);
}
Quaternion EurlToQuaternion(RowVector *Eurl) //欧拉角转四元数
{
	// yaw (Z), pitch (Y), roll (X)
	float roll = Eurl->data[0];//45.0*P_I/180.0;顺序有待验证
    float pitch = Eurl->data[1];//45.0 * P_I / 180.0;
    float yaw = Eurl->data[2];// 90.0 * P_I / 180.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}
RowVector QuaternionToEulerAngles(Quaternion q) //四元数转欧拉角
{
    RowVector angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.data[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.data[1] = copysign(P_I / 2, sinp); // use 90 degrees if out of range
    else
        angles.data[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.data[2] = atan2(siny_cosp, cosy_cosp);

    return angles;
}
RowVector  matrix_vector_multiply(const Matrix* mat, const RowVector* vec) 
{
	RowVector result;
    const uint8_t n = 3;
    float sum;

    // 进入临界区保护（防止任务切换）
    //taskENTER_CRITICAL();
    for (uint8_t i = 0; i < n; ++i) {
        sum = 0.0f;
        for (uint8_t j = 0; j < n; ++j) {
            sum += mat->data[i][j] * vec->data[j];
        }
        result.data[i] = sum;
    }
	return result;
/*    taskEXIT_CRITICAL()*/;

}
RowVector_15  matrix_vector_multiply_15_15(const Matrix_15_15* mat, const RowVector_15* vec) 
{
	RowVector_15 result;
    const uint8_t n = 15;
    float sum;

    // 进入临界区保护（防止任务切换）
    //taskENTER_CRITICAL();
    for (uint8_t i = 0; i < n; ++i) {
        sum = 0.0f;
        for (uint8_t j = 0; j < n; ++j) {
            sum += mat->data[i][j] * vec->data[j];
        }
        result.data[i] = sum;
    }
	return result;
/*    taskEXIT_CRITICAL()*/;

}
 RowVector Add_vector(const RowVector* vec_1,const RowVector* vec_2)
 {
	 int i;
	 int j=3;
	 RowVector sum;
	 for(i=0;i<j;i++)
	 {
		sum.data[i]=vec_1->data[i]+vec_2->data[i];
	 }
	 return sum; 
 }
 RowVector Sub_vector(const RowVector* vec_1,const RowVector* vec_2)
 {
	 int i;
	 int j=3;
	 RowVector sum;
	 for(i=0;i<j;i++)
	 {
		sum.data[i]=vec_1->data[i]-vec_2->data[i];
	 }
	 return sum; 
 }
  Matrix transpose( Matrix *Cnb)//矩阵转置
{
	Matrix M;
	M=*Cnb;
    for (int i = 0; i < 3; i++) {
        // 只遍历上三角部分，避免重复交换
        for (int j = i + 1; j < 3; j++) {
            // 交换元素 matrix[i][j] 和 matrix[j][i]
            float temp = M.data[i][j];
            M.data[i][j] = M.data[j][i];
            M.data[j][i] = temp;
        }
    }
	return M;
}
  void transpose_15_15( Matrix_15_15 *M)//矩阵转置
{

    for (int i = 0; i < 15; i++) {
        // 只遍历上三角部分，避免重复交换
        for (int j = i + 1; j < 15; j++) {
            // 交换元素 matrix[i][j] 和 matrix[j][i]
            float temp = M->data[i][j];
            M->data[i][j] = M->data[j][i];
            M->data[j][i] = temp;
        }
    }

}
RowVector cross_product(RowVector *a, RowVector *b) 
{
    RowVector result;
    result.data[0] = a->data[1]* b->data[2] - a->data[2] * b->data[1];
    result.data[1] = a->data[2] * b->data[0] - a->data[0] * b->data[2];
    result.data[2] = a->data[0] * b->data[1] - a->data[1] * b->data[0];
    return result;
}
Matrix _Askew(RowVector *a,int sign)//反对称矩阵加符号
{
	Matrix M;
	memset(&(M.data),0,sizeof(M.data));
	if(sign==1)
	{
	  M.data[0][1]=-a->data[2];
	  M.data[1][0]=a->data[2];
	  M.data[0][2]=a->data[1];
	  M.data[2][0]=-a->data[1];
	  M.data[1][2]=-a->data[0];
	  M.data[2][1]=a->data[0];
	}
	else
	{
		M.data[0][1]=a->data[2];
		M.data[1][0]=-a->data[2];
		M.data[0][2]=-a->data[1];
		M.data[2][0]=a->data[1];
		M.data[1][2]=a->data[0];
		M.data[2][1]=-a->data[0];
	}

	return M;
	
}
RowVector Number_multiply_vector(float number ,RowVector *a)
{
	RowVector b;
	b.data[0]=a->data[0]*number;
	b.data[1]=a->data[1]*number;
	b.data[2]=a->data[2]*number;
	return b;
}
void Number_multiply_Matrix_3_3(float number ,Matrix *a)
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			a->data[i][j]=a->data[i][j]*number;
		}
	}
}
void Number_multiply_Matrix_15_15(float number ,Matrix_15_15 *a)
{
	if (a == NULL) return;
	for(int i=0;i<15;i++)
	{
		for(int j=0;j<15;j++)
		{
			a->data[i][j]=a->data[i][j]*number;
		}
	}
}
Matrix_15_15 Number_multiply_Matrix_15_15_(float number ,Matrix_15_15 a)
{

	for(int i=0;i<15;i++)
	{
		for(int j=0;j<15;j++)
		{
			a.data[i][j]=a.data[i][j]*number;
		}
	}
	return a;
}
Matrix multiply_3_3(Matrix *A, Matrix *B)
{
    Matrix C;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 3; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
void multiply_15_15(Matrix_15_15 *A, Matrix_15_15 *B,Matrix_15_15 *C)
{

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            C->data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 15; k++) {
                C->data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }

}
Matrix_15_15 multiply_15_15_(Matrix_15_15 *A, Matrix_15_15 *B)
{
    Matrix_15_15 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 15; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix addMatrices_3_3(Matrix *A, Matrix *B) 
{
	Matrix C;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C.data[i][j] = A->data[i][j] + B->data[i][j]; // 对应位置相加
        }
    }
	return C;
}
void addMatrices_15_15(Matrix_15_15 *A, Matrix_15_15 *B,Matrix_15_15 *C) 
{

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            C->data[i][j] = A->data[i][j] + B->data[i][j]; // 对应位置相加
        }
    }

}
Matrix_15_15 SubMatrices_15_15(Matrix_15_15 *A, Matrix_15_15 *B) 
{
	Matrix_15_15 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            C.data[i][j] = A->data[i][j] - B->data[i][j]; // 对应位置相加
        }
    }
	return C;
}
void Identity_matrix_15_15(Matrix_15_15 *M)
{
//	Matrix_15_15 M;
//	memset(&M,0,sizeof(Matrix_15_15));
	int i=0;
	for(;i<15;i++)
	{
		M->data[i][i]=1.0;
	}

}
float Vector_Module_2(RowVector *a)
{
	return (pow(a->data[0],2)+pow(a->data[1],2)+pow(a->data[2],2));
}
void Add_Matrix_Updata(Matrix*M,uint8_t row,uint8_t column)
{
	uint8_t i=row;
	uint8_t j=column;
	for(;i<3+row;i++)
	{
		for(;j<3+column;j++)
		{
			Kf.Ft.data[i][j]=M->data[i-row][j-column];
		}
		j = column;
	}	
}
void State_Matrix_Updata(Matrix*Maa,Matrix*Mav,Matrix*Map,Matrix*Mva,Matrix*Mvv,Matrix*Mvp,Matrix*Mpv,Matrix*Mpp)
{
	Matrix cbn;
	Matrix _cbn;
	Add_Matrix_Updata(Maa,0,0);
	Add_Matrix_Updata(Mav,0,3);
	Add_Matrix_Updata(Map,0,6);
	Add_Matrix_Updata(Mva,3,0);
	Add_Matrix_Updata(Mvv,3,3);
	Add_Matrix_Updata(Mvp,3,6);
	Add_Matrix_Updata(Mpv,6,3);
	Add_Matrix_Updata(Mpp,6,6);
	cbn=Ins.Dcm;
	_cbn=cbn;
	Number_multiply_Matrix_3_3(-1,&_cbn);
	Add_Matrix_Updata(&_cbn,0,9);
	Add_Matrix_Updata(&cbn,3,12);
}
Quaternion Quaternion_Update(RowVector *W,Quaternion Q,int Direction)//旋转矢量转四元数,1代表左乘四元数对应体坐标系，-1代表右乘
{
	Quaternion Q_d;
	Quaternion Qre;
	float Module_2;
	double n;
	double n_2;
	double c;
	double s;
	Module_2=Vector_Module_2(W);
	if(Module_2<1.0E-7)
	{
		c=1-Module_2*(1/8-Module_2/384);
		s = 0.5 - Module_2*(1/48-Module_2/3840);
	}
	else
	{
	  n=sqrt(Module_2);
	  n_2=n/2;
	  c=cos(n_2);
	  s=sin(n_2)/n;
	}
	Q_d.w=c;
	Q_d.x=s*W->data[0];
	Q_d.y=s*W->data[1];
	Q_d.z=s*W->data[2];
	if(Direction==-1)
	{
		Qre.w=Q.w*Q_d.w-Q.x*Q_d.x-Q.y*Q_d.y-Q.z*Q_d.z;//右乘增量四元数
		Qre.x=Q.w*Q_d.x+Q.x*Q_d.w+Q.y*Q_d.z-Q.z*Q_d.y;
		Qre.y=Q.w*Q_d.y+Q.y*Q_d.w+Q.z*Q_d.x-Q.x*Q_d.z;
		Qre.z=Q.w*Q_d.z+Q.z*Q_d.w+Q.x*Q_d.y-Q.y*Q_d.x;
		Module_2=pow(Qre.w,2.0)+pow(Qre.x,2.0)+pow(Qre.y,2.0)+pow(Qre.z,2.0);
		if(Module_2>1.000001||Module_2<0.9999999)
		{
			Module_2=sqrt(Module_2);
			Qre.w/=Module_2;
			Qre.x/=Module_2;
			Qre.y/=Module_2;
			Qre.z/=Module_2;
		}
		return Qre;
	}
	else
	{
		Quaternion temp;
		temp=Q;
		Q=Q_d;//交换
		Q_d=temp;
		Qre.w=Q.w*Q_d.w-Q.x*Q_d.x-Q.y*Q_d.y-Q.z*Q_d.z;//左乘增量四元数
		Qre.x=Q.w*Q_d.x+Q.x*Q_d.w+Q.y*Q_d.z-Q.z*Q_d.y;
		Qre.y=Q.w*Q_d.y+Q.y*Q_d.w+Q.z*Q_d.x-Q.x*Q_d.z;
		Qre.z=Q.w*Q_d.z+Q.z*Q_d.w+Q.x*Q_d.y-Q.y*Q_d.x;
		Module_2=pow(Qre.w,2.0)+pow(Qre.x,2.0)+pow(Qre.y,2.0)+pow(Qre.z,2.0);
		if(Module_2>1.000001||Module_2<0.9999999)
		{
			Module_2=sqrt(Module_2);
			Qre.w/=Module_2;
			Qre.x/=Module_2;
			Qre.y/=Module_2;
			Qre.z/=Module_2;
		}
		return Qre;
	}
}
Matrix_6_6 matrixInverse(Matrix_6_6 *input )
{
    float aug[6][12]; // 增广矩阵
    int i, j, k, pivot;
    Matrix_6_6 output;
    // 初始化增广矩阵：左边是原矩阵，右边是单位矩阵
    for (i = 0; i < 6; ++i) {
        for (j = 0; j < 6; ++j) {
            aug[i][j] = input->data[i][j];
            aug[i][j + 6] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 高斯-约旦消元
    for (k = 0; k < 6; ++k) {
        // 寻找主元（列中最大元素）
        pivot = k;
        for (i = k + 1; i < 6; ++i) {
            if (aug[i][k] > aug[pivot][k]) {
                pivot = i;
            }
        }

        // 如果主元为0，矩阵不可逆
        //if (aug[pivot][k] == 0.0) {
        //    /*return -1;*/ // 错误码：奇异矩阵

        //}

        // 交换当前行和主元行
        if (pivot != k) {
            for (j = 0; j < 12; ++j) {
                float temp = aug[k][j];
                aug[k][j] = aug[pivot][j];
                aug[pivot][j] = temp;
            }
        }

        // 归一化当前行
        float div = aug[k][k];
        for (j = k; j < 12; ++j) {
            aug[k][j] /= div;
        }

        // 消去其他行在当前列的元素
        for (i = 0; i < 6; ++i) {
            if (i != k) {
                float factor = aug[i][k];
                for (j = k; j < 12; ++j) {
                    aug[i][j] -= factor * aug[k][j];
                }
            }
        }
    }

    // 提取逆矩阵部分
    for (i = 0; i < 6; ++i) {
        for (j = 0; j < 6; ++j) {
            output.data[i][j] = aug[i][j + 6];
        }
    }
    return output;
}
Matrix matrixInverse_3_3(Matrix *input )
{
    float aug[3][6]; // 增广矩阵
    int i, j, k, pivot;
    Matrix output;
    // 初始化增广矩阵：左边是原矩阵，右边是单位矩阵
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            aug[i][j] = input->data[i][j];
            aug[i][j + 3] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 高斯-约旦消元
    for (k = 0; k < 3; ++k) {
        // 寻找主元（列中最大元素）
        pivot = k;
        for (i = k + 1; i < 3; ++i) {
            if (aug[i][k] > aug[pivot][k]) {
                pivot = i;
            }
        }

        // 如果主元为0，矩阵不可逆
        //if (aug[pivot][k] == 0.0) {
        //    /*return -1;*/ // 错误码：奇异矩阵

        //}

        // 交换当前行和主元行
        if (pivot != k) {
            for (j = 0; j < 6; ++j) {
                float temp = aug[k][j];
                aug[k][j] = aug[pivot][j];
                aug[pivot][j] = temp;
            }
        }

        // 归一化当前行
        float div = aug[k][k];
        for (j = k; j < 6; ++j) {
            aug[k][j] /= div;
        }

        // 消去其他行在当前列的元素
        for (i = 0; i < 3; ++i) {
            if (i != k) {
                float factor = aug[i][k];
                for (j = k; j < 6; ++j) {
                    aug[i][j] -= factor * aug[k][j];
                }
            }
        }
    }

    // 提取逆矩阵部分
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            output.data[i][j] = aug[i][j + 3];
        }
    }
    return output;
}
Matrix_15_6 transpose_Matrix_6_15(Matrix_6_15* src)
{
    Matrix_15_6 dst;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 15; ++j) {
            dst.data[j][i] = src->data[i][j]; // 行列索引交换
        }
    }
    return dst;
}
Matrix_15_3 transpose_Matrix_3_15(Matrix_3_15* src)
{
    Matrix_15_3 dst;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 15; ++j) {
            dst.data[j][i] = src->data[i][j]; // 行列索引交换
        }
    }
    return dst;
}
Matrix_15_6 multiply_15_15_15_6(Matrix_15_15 *A, Matrix_15_6 *B)
{
    Matrix_15_6 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 15; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix_15_3 multiply_15_15_15_3(Matrix_15_15 *A, Matrix_15_3 *B)
{
    Matrix_15_3 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 3; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 15; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix_6_6 multiply_6_15_15_6(Matrix_6_15 *A, Matrix_15_6 *B)
{
    Matrix_6_6 C;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 15; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix multiply_3_15_15_3(Matrix_3_15 *A, Matrix_15_3 *B)
{
    Matrix C;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 15; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix_6_6  Matrices_6_6(Matrix_6_6 *A, Matrix_6_6 *B) 
{
	Matrix_6_6 C;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = A->data[i][j] + B->data[i][j]; // 对应位置相加
        }
    }
	return C;
}
Matrix_15_6 multiply_15_6_6_6(Matrix_15_6 *A, Matrix_6_6 *B)
{
    Matrix_15_6 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 6; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix_15_3 multiply_15_3_3_3(Matrix_15_3 *A, Matrix *B)
{
    Matrix_15_3 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 3; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 3; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
RowVector_6  matrix_vector_multiply_6_15(const Matrix_6_15* mat, const RowVector_15* vec) 
{
	RowVector_6 result;
    float sum;
    for (uint8_t i = 0; i < 6; ++i) {
        sum = 0.0f;
        for (uint8_t j = 0; j < 15; ++j) {
            sum += mat->data[i][j] * vec->data[j];
        }
        result.data[i] = sum;
    }
	return result;
}
RowVector  matrix_vector_multiply_3_15(const Matrix_3_15* mat, const RowVector_15* vec) 
{
	RowVector result;
    float sum;
    for (uint8_t i = 0; i < 3; ++i) {
        sum = 0.0f;
        for (uint8_t j = 0; j < 15; ++j) {
            sum += mat->data[i][j] * vec->data[j];
        }
        result.data[i] = sum;
    }
	return result;
}
 RowVector_6 Sub_vector_6(const RowVector_6* vec_1,const RowVector_6* vec_2)
 {
	 int i;
	 int j=6;
	 RowVector_6 sum;
	 for(i=0;i<j;i++)
	 {
		sum.data[i]=vec_1->data[i]-vec_2->data[i];
	 }
	 return sum; 
 }
 RowVector_15  matrix_vector_multiply_15_6(const Matrix_15_6* mat, const RowVector_6* vec) 
{
	RowVector_15 result;
    float sum;
    for (uint8_t i = 0; i < 15; ++i) {
        sum = 0.0f;
        for (uint8_t j = 0; j < 6; ++j) {
            sum += mat->data[i][j] * vec->data[j];
        }
        result.data[i] = sum;
    }
	return result;
}
 RowVector_15  matrix_vector_multiply_15_3(const Matrix_15_3* mat, const RowVector* vec) //指针指向的内容是不可在函数之内修改的
{
	RowVector_15 result;
    float sum;
    for (uint8_t i = 0; i < 15; ++i) {
        sum = 0.0f;
        for (uint8_t j = 0; j < 3; ++j) {
            sum += mat->data[i][j] * vec->data[j];
        }
        result.data[i] = sum;
    }
	return result;
}
 RowVector_15 Add_vector_15(const RowVector_15* vec_1,const RowVector_15* vec_2)
 {
	 int i;
	 RowVector_15 sum;
	 for(i=0;i<15;i++)
	 {
		sum.data[i]=vec_1->data[i]+vec_2->data[i];
	 }
	 return sum; 
 }
Matrix_15_15 multiply_15_6_6_15(Matrix_15_6 *A, Matrix_6_15 *B)
{
    Matrix_15_15 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 6; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix_15_15 multiply_15_3_3_15(Matrix_15_3 *A, Matrix_3_15 *B)
{
    Matrix_15_15 C;
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            C.data[i][j] = 0; // 初始化C[i][j]
            for (int k = 0; k < 3; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j]; // 累加乘积
            }
        }
    }
    return C;
}
Matrix_6_6 addMatrices_6_6(Matrix_6_6 *A, Matrix_6_6 *B) 
{
	Matrix_6_6 C;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = A->data[i][j] + B->data[i][j]; // 对应位置相加
        }
    }
	return C;
}