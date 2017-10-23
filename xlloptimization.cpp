// optimization.cpp - ALGLIB optimization routines
// Copyright (c) 2012 KALX, LLC. All rights reserved. No warranty is made.
#include "algxll.h"
#include "cpp/src/optimization.h"

using namespace alglib;
using namespace xll;

static AddInX xai_minlmstate(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmstate"), _T("MINLMSTATE"))
	.Uncalced()
	.Category(CATEGORY)
	.FunctionHelp(_T("Return a handle to a Levenberg-Marquardt optimizer state."))
);
HANDLEX WINAPI
xll_minlmstate(void)
{
#pragma XLLEXPORT
	HANDLEX h(0);

	try {
		handle<minlmstate> state(new minlmstate());
		ensure (state);

		h = state.get();
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());
	}

	return h;
}

static AddInX xai_minlmsetcond(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmsetcond"), _T("MINLMSETCOND"))
	.Arg(XLL_HANDLEX, _T("State"), _T("is the state."))
	.Arg(XLL_DOUBLEX, _T("EpsG"), _T("is the gradient stopping condition."))
	.Arg(XLL_DOUBLEX, _T("EpsF"), _T("is the function stopping condition."))
	.Arg(XLL_DOUBLEX, _T("EpsX"), _T("is the argument stopping condition."))
	.Arg(XLL_USHORTX, _T("MaxIts"), _T("is the maximum number of iterations to use. "))
	.Category(CATEGORY)
	.FunctionHelp(_T("Set stopping conditions and return the State handle."))
);
HANDLEX WINAPI
xll_minlmsetcond(HANDLEX hstate, double epsg, double epsf, double epsx, unsigned short maxits)
{
#pragma XLLEXPORT

	try {
		handle<minlmstate> state(hstate);
		ensure (state);

		minlmsetcond(*state, epsg, epsf, epsx, maxits);
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return hstate;
}

static AddInX xai_minlmsetbc(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmsetbc"), _T("MINLMSETBC"))
	.Arg(XLL_HANDLEX, _T("State"), _T("is the state."))
	.Arg(XLL_FPX, _T("BndL"), _T("is the array of lower bounds."))
	.Arg(XLL_FPX, _T("BndU"), _T("is the array of upper bounds. "))
	.Category(CATEGORY)
	.FunctionHelp(_T("Set bounds for function arguments."))
);
HANDLEX WINAPI
xll_minlmsetbc(HANDLEX hstate, const xfp* pbndl, const xfp* pbndu)
{
#pragma XLLEXPORT

	try {
		handle<minlmstate> state(hstate);
		ensure (state);

		real_1d_array bndl, bndu;
		bndl.setcontent(size(*pbndl), pbndl->array);
		bndu.setcontent(size(*pbndu), pbndu->array);

		minlmsetbc(*state, bndl, bndu);
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return hstate;
}

static AddInX xai_minlmsetacctype(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmsetacctype"), _T("MINLMSETACCTYPE"))
	.Arg(XLL_HANDLEX, _T("State"), _T("is the state."))
	.Arg(XLL_SHORTX, _T("AccType"), _T("is the acceleration type."))
	.Category(CATEGORY)
	.FunctionHelp(_T("Set acceleration type for solver."))
);
HANDLEX WINAPI
xll_minlmsetacctype(HANDLEX hstate, SHORT acctype)
{
#pragma XLLEXPORT

	try {
		ensure (acctype == 0 || acctype == 1);

		handle<minlmstate> state(hstate);
		ensure (state);

		minlmsetacctype(*state, acctype);
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return hstate;
}

static AddInX xai_minlmreport(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmreport"), _T("MINLMREPORT"))
	.Arg(XLL_HANDLEX, _T("Report"), _T("is an optional handle to a report. "))
	.Uncalced()
	.Category(CATEGORY)
	.FunctionHelp(_T("Return a handle to a Levenberg-Marquardt optimizer report or the report if the handle is nonzero."))
);
HANDLEX WINAPI
xll_minlmreport(HANDLEX hreport)
{
#pragma XLLEXPORT
	HANDLEX h(0);

	try {
		if (hreport) {
			handle<minlmreport> report(hreport);
			ensure (report);

			OPERX o;

			o.push_back(OPERX(_T("TerminationType")));
			switch (report->terminationtype) {
			case -7:
				o.push_back(OPERX(_T("derivative correctness check failed")));
				break;
			case 1:
				o.push_back(OPERX(_T("relative function improvement is no more than EpsF")));
				break;
			case 2:
				o.push_back(OPERX(_T("step is no more than EpsX")));
				break;
			case 4:
				o.push_back(OPERX(_T("gradient is no more than EpsG")));
				break;
			case 5:
				o.push_back(OPERX(_T("MaxIts steps was taken")));
				break;
			case 7:
				o.push_back(OPERX(_T("stopping conditions are too stringent, further improvement is impossible")));
				break;
			default:
				o.push_back(OPERX(_T("unknown termination condition")));
			}

			o.push_back(OPERX(_T("IterationsCount")));
			o.push_back(OPERX(report->iterationscount));

			o.push_back(OPERX(_T("NFunc")));
			o.push_back(OPERX(report->nfunc));

			o.push_back(OPERX(_T("NJac")));
			o.push_back(OPERX(report->njac));

			o.push_back(OPERX(_T("NGrad")));
			o.push_back(OPERX(report->ngrad));

			o.push_back(OPERX(_T("NHess")));
			o.push_back(OPERX(report->nhess));

			o.push_back(OPERX(_T("NCholesky")));
			o.push_back(OPERX(report->ncholesky));

			o.resize(o.size()/2, 2);

			handle<OPERX> ho(new OPERX(o));

			h = ho.get();
		}
		else {
			handle<minlmreport> report(new minlmreport());
			ensure (report);

			h = report.get();
		}
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());
	}

	return h;
}

static AddInX xai_minlmresults(
	FunctionX(XLL_FPX, _T("?xll_minlmresults"), _T("MINLMRESULTS"))
	.Arg(XLL_HANDLEX, _T("State"), _T("is a handle to a MINLMSTATE."))
	.Arg(XLL_HANDLEX, _T("Report"), _T("is a handle to a MINLMREPORT()."))
	.Category(CATEGORY)
	.FunctionHelp(_T("Returns the Report handle and the optimum solution given State."))
);
xfp* WINAPI
xll_minlmresults(HANDLEX hstate, HANDLEX hreport)
{
#pragma XLLEXPORT
	static FPX x;

	try {
		handle<minlmstate> state(hstate);
		ensure (state);
		handle<minlmreport> report(hreport);
		ensure (report);

		real_1d_array x_;

		minlmresults(*state, x_, *report);

		x.reshape(1, 1 + (xword)x_.length());
		x[0] = hreport;
		std::copy(x_.getcontent(), x_.getcontent() + x_.length(), 1 + x.begin());
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return x.get();
}

static AddInX xai_minlmcreatev(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmcreatev"), _T("MINLMCREATEV"))
	.Arg(XLL_SHORTX, _T("N"), _T("is the dimension of the function arguments."))
	.Arg(XLL_SHORTX, _T("M"), _T("is the number of functions in the sum of squares."))
	.Arg(XLL_FPX, _T("X"), _T("is an N dimesional vector with the initial guess."))
	.Arg(XLL_DOUBLEX, _T("Diffstep"), _T("is the differentiation step."))
	.Arg(XLL_HANDLEX, _T("State"), _T("is a handle to the state created by MINLMSTATE. "))
	.Category(CATEGORY)
	.FunctionHelp(_T("Create a Levenberg-Marquardt optimizer using minlmcreatev."))
);
HANDLEX WINAPI
xll_minlmcreatev(SHORT n, SHORT m, const xfp* px, double dx, HANDLEX hstate)
{
#pragma XLLEXPORT

	try {
		ensure (n <= size(*px));

		handle<minlmstate> state(hstate);
		ensure (state);

		real_1d_array x;
		x.setcontent(n, px->array);

		minlmcreatev(n, m, x, dx, *state);
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return hstate;
}

static AddInX xai_minlmcreatevj(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmcreatevj"), _T("MINLMCREATEVJ"))
	.Arg(XLL_SHORTX, _T("N"), _T("is the dimension of the function arguments."))
	.Arg(XLL_SHORTX, _T("M"), _T("is the number of functions in the sum of squares."))
	.Arg(XLL_FPX, _T("X"), _T("is an N dimesional vector with the initial guess."))
	.Arg(XLL_HANDLEX, _T("State"), _T("is a handle to the state created by MINLMSTATE. "))
	.Category(CATEGORY)
	.FunctionHelp(_T("Create a Levenberg-Marquardt optimizer using minlmcreatevj."))
);
HANDLEX WINAPI
xll_minlmcreatevj(SHORT n, SHORT m, const xfp* px, HANDLEX hstate)
{
#pragma XLLEXPORT

	try {
		ensure (n <= size(*px));

		handle<minlmstate> state(hstate);
		ensure (state);

		real_1d_array x;
		x.setcontent(n, px->array);

		minlmcreatevj(n, m, x, *state);
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return hstate;
}

// ptr is a pointer to the regid of a UDF
void func(const real_1d_array& x, double& fi, void* ptr)
{
	OPERX x_((xword)x.length(), 1);

	std::copy(x.getcontent(), x.getcontent() + x.length(), x_.begin());

	OPERX f_ = ExcelX(xlUDF, OPERX(*(double*)ptr), x_);
	ensure (f_.xltype == xltypeNum);

	fi = f_.val.num;
}
void grad(const real_1d_array& x, real_1d_array& fi, void* ptr)
{
	OPERX x_((xword)x.length(), 1);

	std::copy(x.getcontent(), x.getcontent() + x.length(), x_.begin());

	OPERX f_ = ExcelX(xlUDF, OPERX(*(double*)ptr), x_);

	std::copy(f_.begin(), f_.end(), fi.getcontent());
}

void fvec(const real_1d_array& x, real_1d_array& fi, void* ptr)
{
	OPERX x_((xword)x.length(), 1);

	std::copy(x.getcontent(), x.getcontent() + x.length(), x_.begin());

	OPERX f_ = ExcelX(xlUDF, OPERX(*(double*)ptr), x_);

	std::copy(f_.begin(), f_.end(), fi.getcontent());
}
/// ???does fi really need to be specified again???
void jac(const real_1d_array& x, real_1d_array& fi, real_2d_array& fjac, void* ptr)
{
	OPERX x_((xword)x.length(), 1);

	std::copy(x.getcontent(), x.getcontent() + x.length(), x_.begin());

	std::pair<double,double> vj = *(std::pair<double,double>*)ptr;

	OPERX v_ = ExcelX(xlUDF, OPERX(vj.first), x_);
	OPERX j_ = ExcelX(xlUDF, OPERX(vj.second), x_);

	ensure (v_.size() == j_.rows());
	ensure (v_.size() == j_.columns());

	std::copy(v_.begin(), v_.end(), fi.getcontent());
	for (xword i = 0; i < j_.rows(); ++i)
		for (xword j = 0; j < j_.columns(); ++j)
			fjac[i][j] = j_(i, j);
}

static AddInX xai_minlmoptimize(
	FunctionX(XLL_HANDLEX, _T("?xll_minlmoptimize"), _T("MINLMOPTIMIZE"))
	.Arg(XLL_HANDLEX, _T("State"), _T("is a handle to the state created by MINLMCREATE."))
	.Arg(XLL_HANDLEX, _T("Func"), _T("is a handle to a user defined function."))
	.Arg(XLL_HANDLEX, _T("Jac"), _T("is a handle to a user defined Jacobian."))
	//.Arg(XLL_HANDLEX, _T("Hess"), _T("is a handle to a user defined Hessian."))
	.Category(CATEGORY)
	.FunctionHelp(_T("Create a Levenberg-Marquardt optimizer using minlmoptimize."))
);
HANDLEX WINAPI
xll_minlmoptimize(HANDLEX hstate, HANDLEX hfunc, HANDLEX hjac)
{
#pragma XLLEXPORT

	try {
		handle<minlmstate> state(hstate);
		ensure (state);

		if (hjac) {
			std::pair<double,double> vj(hfunc, hjac);
			minlmoptimize(*state, fvec, jac, 0, (void*)&vj);
		}
		else {
			minlmoptimize(*state, fvec, 0, (void*)&hfunc);
		}
	}
	catch (const std::exception& ex) {
		XLL_ERROR(ex.what());

		return 0;
	}

	return hstate;
}

static AddInX xai_function1_fvec(
	FunctionX(XLL_FPX, _T("?xll_function1_fvec"), _T("FUNCTION1.FVEC"))
	.Arg(XLL_FPX, _T("x"), _T("is an array of two numbers. "))
);
xfp* WINAPI
xll_function1_fvec(xfp* px)
{
#pragma XLLEXPORT
	static FPX f(2,1);

	f[0] = 10*pow(px->array[0]+3,2);
	f[1] = pow(px->array[1]-3,2);

	return f.get();
}
static AddInX xai_function1_jac(
	FunctionX(XLL_FPX, _T("?xll_function1_jac"), _T("FUNCTION1.JAC"))
	.Arg(XLL_FPX, _T("x"), _T("is an array of two numbers. "))
);
xfp* WINAPI
xll_function1_jac(xfp* px)
{
#pragma XLLEXPORT
	static FPX jac(2,2);

    jac(0,0) = 20*(px->array[0]+3);
    jac(0,1) = 0;
    jac(1,0) = 0;
    jac(1,1) = 2*(px->array[1]-3);


	return jac.get();
}