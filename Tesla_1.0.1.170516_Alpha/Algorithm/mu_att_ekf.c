/* Include Files */
#include "imu.h"
#include "hardware.h"

/* Function Declarations */
float norm3( float x[3]);

/* Function Definitions */
/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
float norm3(float x[3])
{
//  float y;
//  float scale;
//  int k;
//  float absxk;
//  float t;
//  y = 0.0F;
//  scale = 1.17549435E-38F;
//  for (k = 0; k < 3; k++) {
//    absxk = (float)fabs(x[k]);
//    if (absxk > scale) {
//      t = scale / absxk;
//      y = 1.0F + y * t * t;
//      scale = absxk;
//    } else {
//      t = absxk / scale;
//      y += t * t;
//    }
//  }

//  return scale * (float)sqrt(y);
	float norm3_value;	
	
	norm3_value = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
	if(isnan(norm3_value)|(norm3_value==0))
		rt_kprintf("\r\n norm3 output need to be check.......");
	
	return norm3_value;
}

/*
 * jocobian matrix
 * Arguments    : float P[16]
 *                const float acc_imu_g[3]
 *                float stateVec[4]
 *                const float R[9]
 * Return Type  : void
 */
float att_mu_acc(float P[16], imu_scaled_body *imu, float quat[4], float R[9])
{
  float b_stateVec[12];
  float H[12];
  int rtemp;
  int k;
  float Y[3];
  float maxval;
  float c[9];
  float b_c[12];
  int r1;
  int r2;
  int r3;
  float a21;
  float K[12];
  float b_acc_imu_g[3];
  float fv1[16];
  float fv2[16];
  static const signed char iv0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };
  float trace_P;
  int i;
	float acc_imu_g[3];
		
	acc_imu_g[0] = imu->acc_m_s2[0];
	acc_imu_g[1] = imu->acc_m_s2[1];
	acc_imu_g[2] = imu->acc_m_s2[2];
		

  /* 'mu_att_ekf:4' H = 2*[-stateVec(3) stateVec(4) -stateVec(1) stateVec(2);... */
  /* 'mu_att_ekf:5'        stateVec(2) stateVec(1) stateVec(4) stateVec(3);... */
  /* 'mu_att_ekf:6'        stateVec(1) -stateVec(2) -stateVec(3) stateVec(4)]; */
  b_stateVec[0] = -quat[2];
  b_stateVec[3] = quat[3];
  b_stateVec[6] = -quat[0];
  b_stateVec[9] = quat[1];
  b_stateVec[1] = quat[1];
  b_stateVec[4] = quat[0];
  b_stateVec[7] = quat[3];
  b_stateVec[10] = quat[2];
  b_stateVec[2] = quat[0];
  b_stateVec[5] = -quat[1];
  b_stateVec[8] = -quat[2];
  b_stateVec[11] = quat[3];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      H[k + 3 * rtemp] = 2.0F * b_stateVec[k + 3 * rtemp];
    }
  }
	if(isnan(quat[0])&&isnan(quat[1])&&isnan(quat[2])&&isnan(quat[3]))
		rt_kprintf("gx = %d",g_sensor_info.mpu.gx);
		
  /* estimated x */
  /* 'mu_att_ekf:8' Y = H*stateVec; */
  for (rtemp = 0; rtemp < 3; rtemp++) {
    Y[rtemp] = 0.0F;
    for (k = 0; k < 4; k++) {
      Y[rtemp] += H[rtemp + 3 * k] * quat[k];
    }
  }

  /* 'mu_att_ekf:9' Y = Y/norm(Y); */
  maxval = norm3(Y);
  //
  for (rtemp = 0; rtemp < 3; rtemp++) {
    Y[rtemp] /= maxval;
  }

  /* kalman filter */
  /* 'mu_att_ekf:11' K = P * H'/((H * P * H' + R )); */
  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 3; k++) {
      b_c[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        b_c[rtemp + (k << 2)] += P[rtemp + (r1 << 2)] * H[k + 3 * r1];
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 4; k++) {
      b_stateVec[rtemp + 3 * k] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        b_stateVec[rtemp + 3 * k] += H[rtemp + 3 * r1] * P[r1 + (k << 2)];
      }
    }

    for (k = 0; k < 3; k++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        maxval += b_stateVec[rtemp + 3 * r1] * H[k + 3 * r1];
      }
      c[rtemp + 3 * k] = maxval + R[rtemp + 3 * k];
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
	maxval = (float)fabs(c[0]);
  a21 = (float)fabs(c[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if ((float)fabs(c[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  c[r2] /= c[r1];
  c[r3] /= c[r1];
  c[3 + r2] -= c[r2] * c[3 + r1];
  c[3 + r3] -= c[r3] * c[3 + r1];
  c[6 + r2] -= c[r2] * c[6 + r1];
  c[6 + r3] -= c[r3] * c[6 + r1];
  if ((float)fabs(c[3 + r3]) > (float)fabs(c[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  c[3 + r3] /= c[3 + r2];
  c[6 + r3] -= c[3 + r3] * c[6 + r2];
  for (k = 0; k < 4; k++) {
    K[k + (r1 << 2)] = b_c[k] / c[r1];
    K[k + (r2 << 2)] = b_c[4 + k] - K[k + (r1 << 2)] * c[3 + r1];
    K[k + (r3 << 2)] = b_c[8 + k] - K[k + (r1 << 2)] * c[6 + r1];
    K[k + (r2 << 2)] /= c[3 + r2];
    K[k + (r3 << 2)] -= K[k + (r2 << 2)] * c[6 + r2];
    K[k + (r3 << 2)] /= c[6 + r3];
    K[k + (r2 << 2)] -= K[k + (r3 << 2)] * c[3 + r3];
    K[k + (r1 << 2)] -= K[k + (r3 << 2)] * c[r3];
    K[k + (r1 << 2)] -= K[k + (r2 << 2)] * c[r2];
  }

  /* measurement */
  /* 'mu_att_ekf:13' measurement_acc = -acc_imu_g/norm(acc_imu_g); */
  maxval = norm3(acc_imu_g);
	if(isnan(maxval)){
		rt_kprintf("\r\nnorm error!!!!!!!");
	  rt_kprintf("\r\n acc_imu_g=[%d,%d,%d]",(int)acc_imu_g[0],(int)acc_imu_g[1],(int)acc_imu_g[2]);
	}

  /* state vector correction */
  /* 'mu_att_ekf:15' stateVec = stateVec + K * (measurement_acc - Y); */
  for (rtemp = 0; rtemp < 3; rtemp++) {
    b_acc_imu_g[rtemp] = -acc_imu_g[rtemp] / maxval - Y[rtemp];
  }

  /* P matrix correction */
  /* 'mu_att_ekf:17' P = (eye(length(P)) - K *  H)* P; */
  for (rtemp = 0; rtemp < 4; rtemp++) {
    maxval = 0.0F;
    for (k = 0; k < 3; k++) {
      maxval += K[rtemp + (k << 2)] * b_acc_imu_g[k];
    }
		
//		if(isnan(maxval))
//			rt_kprintf("norm error!!!!!!!");

    quat[rtemp] += maxval;
//		if(isnan(quat[0])&&isnan(quat[1])&&isnan(quat[2])&&isnan(quat[3]))
//			rt_kprintf("/r/ngx = %d",g_sensor_info.mpu.gx);
		
    for (k = 0; k < 4; k++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 3; r1++) {
        maxval += K[rtemp + (r1 << 2)] * H[r1 + 3 * k];
      }
      fv1[rtemp + (k << 2)] = (float)iv0[rtemp + (k << 2)] - maxval;
    }
//		if(isnan(maxval))
//			rt_kprintf("norm error!!!!!!!");

    for (k = 0; k < 4; k++) {
      fv2[rtemp + (k << 2)] = 0.0F;
      for (r1 = 0; r1 < 4; r1++) {
        fv2[rtemp + (k << 2)] += fv1[rtemp + (r1 << 2)] * P[r1 + (k << 2)];
      }
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 4; k++) {
      P[k + (rtemp << 2)] = fv2[k + (rtemp << 2)];
    }
  }

  /* guarantee P ,prevent filter diverse due to illness of P matrix */
  /* 'mu_att_ekf:19' P = 0.5*(P + transpose(P)); */
  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 4; k++) {
      fv1[k + (rtemp << 2)] = 0.5F * (P[k + (rtemp << 2)] + P[rtemp + (k << 2)]);
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    for (k = 0; k < 4; k++) {
      P[k + (rtemp << 2)] = fv1[k + (rtemp << 2)];
    }
  }

  /* guarantee main diagonal elements positive */
  /* 'mu_att_ekf:21' for i=1:length(P) */
  for (i = 0; i < 4; i++) {
    /* 'mu_att_ekf:22' if P(i,i) < 0 */
    if (P[i + (i << 2)] < 0.0F) {
      /* 'mu_att_ekf:23' P(i,i) = 0; */
      P[i + (i << 2)] = 0.0F;
    }
  }
	
		return trace_P = P[0]+P[5]+P[10]+P[15];

}
