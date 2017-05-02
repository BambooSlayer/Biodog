// XWRobotDlg.h : header file
//

#include "afxwin.h"
#include "cmscomm.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <iostream>
#include <windows.h>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <windows.h>

using namespace std;
using namespace cv;
//#define encoder_set
#if !defined(AFX_XWROBOTDLG_H__05841B20_0F57_49C1_A09D_3CD93A028A04__INCLUDED_)
#define AFX_XWROBOTDLG_H__05841B20_0F57_49C1_A09D_3CD93A028A04__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CXWRobotDlg dialog

class CXWRobotDlg : public CDialog
{
// Construction
public:
	CXWRobotDlg(CWnd* pParent = NULL);	// standard constructor
// Dialog Data
	//{{AFX_DATA(CXWRobotDlg)
	enum { IDD = IDD_XWROBOT_DIALOG };
	CSliderCtrl	m_RH;
	CSliderCtrl	m_RF;
	CSliderCtrl	m_LH;
	CSliderCtrl	m_LF;
	CButton	m_Pause;
	CButton	m_Continue;
	CButton	m_Walk;
	CButton	m_Trot;
	CButton	m_Stop;
	CButton	m_ModeSettings;
	CButton	m_AEnable;
	CButton	m_ADisable;
	CButton	m_OpenDevice;
	CButton	m_MoveRelative;
	CButton	m_MoveAbsolute;
	CButton	m_Halt;
	CButton	m_Enable;
	CButton	m_Disable;
	CButton	m_DeviceSettings;
	DWORD	m_AVelocity;
	DWORD	m_DVelocity;
	UINT	m_NodeID;
	long	m_RTPosition;
	long	m_StartPosition;
	long	m_Target;
	DWORD	m_Velocity;
	long	m_HipBacklash;
	long	m_KneeBacklash;
	int		m_Period;
	double	F1,F2,F3,F4,C1,C2,C3,G1,G2,G3,A1,A2,A3;
	int		T1,T2,T3,T4,T5,T6,T7,T8;
	double	m_a;
	double	m_b;
	double	m_c;
	double	m_SW;
	double	m_Amplitude;
	long	m_Hip1;
	long	m_Knee1;
	double	m_h;
	ofstream f1;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CXWRobotDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CXWRobotDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnChangeEDITNodeID();
	afx_msg void OnChangeEDITTarget();
	afx_msg void OnChangeEDITVelocity();
	afx_msg void OnChangeEDITAVelocity();
	afx_msg void OnChangeEDITDVelocity();
	afx_msg void OnBUTTONOpenDevice();
	afx_msg void OnBUTTONDeviceSettings();
	afx_msg void OnBUTTONEnable();
	afx_msg void OnBUTTONDisable();
	afx_msg void OnBUTTONMoveRelative();
	afx_msg void OnBUTTONAbsolute();
	afx_msg void OnBUTTONHalt();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnDestroy();
	afx_msg void OnBUTTONSolo();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnChangeEDITAmplitude();
	afx_msg void OnChangeEDITHipBacklash();
	afx_msg void OnChangeEDITKneeBacklash();
	afx_msg void OnChangeEDITPeriod();
	afx_msg void OnBUTTONModeSettings();
	afx_msg void OnBUTTONADisable();
	afx_msg void OnBUTTONAEnable();
	afx_msg void OnBUTTONWalk();
	afx_msg void OnBUTTONTrot();
	afx_msg void OnBUTTONStop();
	afx_msg void OnBUTTONPause();
	afx_msg void OnBUTTONContinue();
	afx_msg void OnChangeEditH();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

private:
	BOOL m_oUpdateActive;
	BOOL m_open;
    BOOL m_oImmediately;
	HANDLE m_KeyHandle;
	DWORD m_dErrorCode;
	CRect m_rectSmall;
	CRect m_rectLarge;
	CBrush m_brush;
	UINT PauseIndex;
	UINT FirstIndex;
	int i,j;
	//double SWLFK[40],SWRFK[40],SWLHK[40],SWRHK[40];
	//double WLFK[80],WRFK[80],WLHK[80],WRHK[80];//因为我后来又定义过，这里如果保留的话就会溢出
	double STLFK[20],STRFK[20],STLHK[20],STRHK[20];
	double TLFK[20],TRFK[20],TLHK[20],TRHK[20];
	
private:
	BOOL ShowErrorInformation(DWORD dErrorCode);
	BOOL UpdateStatus();
	void Algorithm();
	void FormatTime(time_t time1, char *szTime);
	//static const double SWalkLF[40];
public:
	afx_msg void OnEnChangeEditStartposition();
	afx_msg void OnEnChangeEditknee1();
	afx_msg void OnEnChangeEditknee2();
	afx_msg void OnBnClickedButtonStm32();
	afx_msg void OnEnChangeEdit1();
private:
	CString e_EditReceive;
	CEdit m_EditSend;
public:
	CComboBox m_comb1;
	CComboBox m_comb2;
	CMSComm m_mscom;
	afx_msg void OnBnClickedButtonOpen();
	afx_msg void OnBnClickedButtonSend();
	afx_msg void OnBnClickedButtonClean();
	afx_msg void OnBnClickedButtonClose();
	DECLARE_EVENTSINK_MAP()
	void OnCommMscomm1();
	afx_msg void OnEnChangeEditRtposition();
	afx_msg void OnBnClickedButtonRemember();
	afx_msg void OnBnClickedButtonAbsolute();
	long m_LFH;
	long m_LFK;
	long m_LHH;
	long m_LHK;
	long m_RFH;
	long m_RFK;
//	CEdit m_RHH;
	long m_RHH;
	long m_RHK;
	afx_msg void OnClickedButtonMoverelative2();
	afx_msg void OnClickedButtonAbsolute2();
	CButton m_MoveRelative2;
	CButton m_MoveAbsolute2;
	afx_msg void OnBnClickedButtonAbsolute3();
	afx_msg void OnBnClickedButtonMoverelative3();
	afx_msg void OnBnClickedButtonMoverelative5();
	afx_msg void OnBnClickedButtonMoverelative6();
	afx_msg void OnBnClickedButtonMoverelative13();
	afx_msg void OnBnClickedButtonMoverelative9();
	afx_msg void OnBnClickedButtonMoverelative17();
	afx_msg void OnBnClickedButtonMoverelative14();
	afx_msg void OnBnClickedButtonMoverelative7();
	afx_msg void OnBnClickedButtonMoverelative10();
	afx_msg void OnBnClickedButtonMoverelative18();
	afx_msg void OnBnClickedButtonMoverelative15();
	afx_msg void OnBnClickedButtonMoverelative11();
	afx_msg void OnBnClickedButtonMoverelative19();
	afx_msg void OnBnClickedButtonMoverelative8();
	afx_msg void OnBnClickedButtonMoverelative16();
	afx_msg void OnBnClickedButtonMoverelative12();
	afx_msg void OnBnClickedButtonMoverelative20();
	afx_msg void OnBnClickedButtonMoverelative21();
	CSliderCtrl m_pit;
	CSliderCtrl m_roll;
	long m_pitnum;
//	CEdit m_rollnum;
	long m_rollnum;
	afx_msg void OnBnClickedCam();
};
//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_XWROBOTDLG_H__05841B20_0F57_49C1_A09D_3CD93A028A04__INCLUDED_)
