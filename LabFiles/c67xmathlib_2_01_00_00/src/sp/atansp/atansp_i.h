/* ======================================================================= */
/* atansp_i.h - single precision floating point arctangent                 */
/*              optimized inlined C Implementation (w/ Intrinsics)         */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2010 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

#ifndef ATANSP_I_
#define ATANSP_I_ 1

static inline float atan2f_atan2f_sr2i1(float b, int at2, float a)
{

	float	bg;
	float	g;

	/* calculate 1/b and multiply by a if called from atan2f */
	g = _rcpsp(b);
	g = g*(2.0f - b*g);
	bg = b*g;
	if(at2) g = g*a;
	g = g*(2.0f - bg);	
	return(g);
}

static inline float atan2f_atan2f_sr1i1(float g, float pih, int s, int bn, int an)
{

	float	c1 =  0.00230158202f;
	float	c2 = -0.01394551000f;
	float	c3 =  0.03937087815f;
	float	c4 = -0.07235669163f;
	float	c5 =  0.10521499322f;
	float	c6 = -0.14175076797f;
	float	c7 =  0.19989300877f;
	float	c8 = -0.33332930041f;
	float 	coef;
	float	g2;
	float	g4;
	float	g6;
	float	g8;
	float	g10;
	float	g12;
	int 	ns_nbn;
	float	pol;
	float	tmp1;
	float	tmp2;
	
	/* get coef based on the flags */
	coef = pih;
	if(!s) coef = 3.1415927f;
    ns_nbn = s | bn;
    if(!ns_nbn) coef = 0;
    if(an) coef = -coef;
 
	/* calculate polynomial */
	g2 = g*g;
	g4 = g2*g2;
	g6 = g2*g4;
	g8 = g4*g4;
	g10 = g6*g4;
	g12 = g8*g4;

	tmp1 =  ((c5 * g8) + (c6 * g6)) + ((c7 * g4) + (c8 * g2));
	tmp2 =  (((c1 * g4 + c2 * g2) + c3) * g12) + (c4 * g10);

	pol = tmp1 + tmp2;
	pol =  pol*g + g;

	return( s ? (coef - pol) : (coef + pol));
}

static inline float atansp_i(float a)
{

	
	int		an;
	float	g;
	//float	pih = 1.570796327f;
	float	res;
	int		s = 0;
	float	temp=1;

	//if (a == 0.0f ) return 0.0f;
	//if (b == 0.0f ) return (a>0.0f? pih:-pih); 

	an = (a<0) ? 1 : 0;   /* flag for a negative */

	/* swap a and b before calling division sub routine if a>b */
	if(_fabsf(a)>1.0f){
    	//temp = 1;
		temp = a;
		a = 1;
		s = 1;         /* swap flag */
	}

	g = atan2f_atan2f_sr2i1(temp, 1, a);
	
	/* do polynomial estimation */
	res = atan2f_atan2f_sr1i1(g, 1.57079632679f, s, 0, an);

	if (a == 0.0f ) res = 0.0f;
	
	return (res);

}



#endif


/* ======================================================================== */
/*  End of file: atansp_c.h                                                 */
/* ------------------------------------------------------------------------ */
/*          Copyright (C) 2010 Texas Instruments, Incorporated.             */
/*                          All Rights Reserved.                            */
/* ======================================================================== */
