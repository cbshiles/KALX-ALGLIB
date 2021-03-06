// algxll.h - ALGLIB wrapper for Excel
// Copyright (c) 2012 KALX, LLC. All rights reserved. No warranty is made.
// Uncomment the following line to use features for Excel2007 and above.
//#define EXCEL12
#include "xll/xll.h"

#ifndef CATEGORY
#define CATEGORY _T("ALGXLL")
#endif

typedef xll::traits<XLOPERX>::xcstr xcstr; // pointer to const string
typedef xll::traits<XLOPERX>::xword xword; // use for OPER and FP indices
typedef xll::traits<XLOPERX>::xfp xfp; // use for OPER and FP indices