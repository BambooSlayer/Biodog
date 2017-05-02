// XWRobotDlg.cpp : implementation file
//

#include "stdafx.h"
#include "XWRobot.h"
#include "XWRobotDlg.h"
#include "Definitions.h"
#include "math.h"


const double pi=3.141592654;
const int looptmax=2;
#define ver 0
//ver=0 是第一篇论文的walk方式，无改动版
//ver=1 walk步态抬腿前站稳，还原收起的knee的同时，待抬腿的对角knee先收起5°。（观察一下，是否能和预想一样调整重心。
long LFHS=0;
long LFKS=0;
long LHHS=0;
long LHKS=0;
long RFHS=0;
long RFKS=0;
long RHHS=0;
long RHKS=0;
int loopt=0;//行走周期
int stop_time=0;
//设定初始值
# if ver==0
#include "ANG0.h"//小跨度,无补偿（晃动的还是比较厉害，不敢走
#elif ver==1
#include "ANG1.h"//小跨度，对角补偿8度的版本（可以走，大概
#elif ver==12
#include "ANG1_2.h"//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==121
#include "ANG1_2_1.h"//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==1211
#include "ANG1_2_1_1.h"//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==1212
#include "ANG1_2_1_2.h"//添加了SRHK后登//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==1213
#include "ANG1_2_1_3.h"//添加了SRHK后登//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==1214
#include "ANG1_2_1_4.h"//同时添加了RF的反坐//添加了SRHK后登//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==1215
#include "ANG1_2_1_5.h"//同时添加了所有的反坐//添加了SRHK后登//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==1216
#include "ANG1_2_1_6.h"//加快50%回复姿态的速度//同时添加了所有的反坐//添加了SRHK后登//添加了后腿的姿态补偿（此处是*1.5的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==122
#include "ANG1_2_2.h"//添加了后腿的姿态补偿（此处是*2的补偿//小跨度，对角补偿6+2(Knee+Hip)度的版本
#elif ver==2
#include "ANG2.h"//小跨度，对边补偿8度的版本(还是三没支撑住
#elif ver==3
#include "ANG3.h"//小跨度，对边补偿6度的版本（以上全部都是knee补偿（这个版本会不稳（见手机第一个视频
#elif ver==4
#include "ANG3.h"//尝试原地踏步

#endif

//changed

double STrotLF[20]={0,0,-0.002,-0.004,-0.004,-0.004,-0.002, 0.002, 0.004, 0.003,-0.003,-0.018,-0.061,-0.106,-0.151,-0.180,-0.187,-0.168,-0.129,-0.064};
double STrotRF[20]={0,0,-0.018,-0.040,-0.056,-0.072,-0.090,-0.094,-0.081,-0.047,-0.010, 0.038, 0.120, 0.170, 0.205, 0.219, 0.212, 0.186, 0.143, 0.075};
double STrotLH[20]={0,0,-0.006,-0.017,-0.027,-0.038,-0.053,-0.064,-0.064,-0.048,-0.023, 0.017, 0.096, 0.152, 0.195, 0.216, 0.214, 0.192, 0.152, 0.075};
double STrotRH[20]={0,0, 0.001, 0.001, 0.002, 0.003, 0.006, 0.009, 0.008, 0.002,-0.010,-0.032,-0.086,-0.134,-0.174,-0.195,-0.191,-0.166,-0.123,-0.064};

double TrotLF[20]={ 0.011, 0.073, 0.130, 0.178, 0.216, 0.228, 0.216, 0.188, 0.152, 0.071,-0.010,-0.073,-0.131,-0.178,-0.216,-0.227,-0.216,-0.185,-0.140,-0.082};
double TrotRF[20]={-0.011,-0.073,-0.130,-0.178,-0.216,-0.228,-0.216,-0.188,-0.152,-0.071, 0.010, 0.073, 0.131, 0.178, 0.216, 0.227, 0.216, 0.185, 0.140, 0.082};
double TrotLH[20]={-0.011,-0.073,-0.130,-0.178,-0.216,-0.228,-0.216,-0.188,-0.152,-0.071, 0.010, 0.073, 0.131, 0.178, 0.216, 0.227, 0.216, 0.185, 0.140, 0.082};
double TrotRH[20]={ 0.011, 0.073, 0.130, 0.178, 0.216, 0.228, 0.216, 0.188, 0.152, 0.071,-0.010,-0.073,-0.131,-0.178,-0.216,-0.227,-0.216,-0.185,-0.140,-0.082};
/////////////////////////////////////////////////////////////////////////////
double STrotLF1[20]={0,0,-0.002,-0.004,-0.004,-0.004,-0.002, 0.002, 0.004, 0.003,-0.003,-0.018,-0.061,-0.106,-0.151,-0.180,-0.187,-0.168,-0.129,-0.064};
double STrotRF1[20]={0,0,-0.018,-0.040,-0.056,-0.072,-0.090,-0.094,-0.081,-0.047,-0.010, 0.038, 0.120, 0.170, 0.205, 0.219, 0.212, 0.186, 0.143, 0.075};
double STrotLH1[20]={0,0,-0.006,-0.017,-0.027,-0.038,-0.053,-0.064,-0.064,-0.048,-0.023, 0.017, 0.096, 0.152, 0.195, 0.216, 0.214, 0.192, 0.152, 0.075};
double STrotRH1[20]={0,0, 0.001, 0.001, 0.002, 0.003, 0.006, 0.009, 0.008, 0.002,-0.010,-0.032,-0.086,-0.134,-0.174,-0.195,-0.191,-0.166,-0.123,-0.064};

double TrotLF1[20]={ 0.011, 0.073, 0.130, 0.178, 0.216, 0.228, 0.216, 0.188, 0.152, 0.071,-0.010,-0.073,-0.131,-0.178,-0.216,-0.227,-0.216,-0.185,-0.140,-0.082};
double TrotRF1[20]={-0.011,-0.073,-0.130,-0.178,-0.216,-0.228,-0.216,-0.188,-0.152,-0.071, 0.010, 0.073, 0.131, 0.178, 0.216, 0.227, 0.216, 0.185, 0.140, 0.082};
double TrotLH1[20]={-0.011,-0.073,-0.130,-0.178,-0.216,-0.228,-0.216,-0.188,-0.152,-0.071, 0.010, 0.073, 0.131, 0.178, 0.216, 0.227, 0.216, 0.185, 0.140, 0.082};
double TrotRH1[20]={ 0.011, 0.073, 0.130, 0.178, 0.216, 0.228, 0.216, 0.188, 0.152, 0.071,-0.010,-0.073,-0.131,-0.178,-0.216,-0.227,-0.216,-0.185,-0.140,-0.082};

// CAboutDlg dialog used for App About

char str_nam[34];//保存的文件名

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CXWRobotDlg dialog

CXWRobotDlg::CXWRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CXWRobotDlg::IDD, pParent)
	, e_EditReceive(_T(""))
{
	//{{AFX_DATA_INIT(CXWRobotDlg)
	m_AVelocity = 10000;
	m_DVelocity = 10000;
	m_NodeID = 1;
	m_RTPosition = 0;
	m_StartPosition = 0;
	m_Target = 20000;
	m_Velocity = 1000;
	m_HipBacklash = 1000;
	m_KneeBacklash = 1000;     
	m_Period = 4;
	m_a = -20.6988;
	m_b = 46.5806;
	m_c = -25.2062;
	//AM Coefficients
	m_Amplitude = 1;//1.0;
	m_SW =250*(cos(0.3584*pi-0.22*m_Amplitude)-cos(0.3584*pi+0.22*m_Amplitude))-150*(sqrt(1-pow((33-25*sin(0.3584*pi-0.22*m_Amplitude))/15,2))-sqrt(1-pow((33-25*sin(0.3584*pi+0.22*m_Amplitude))/15,2)));		
	//SW: Stride Width
	m_Hip1 = 0;
	m_Knee1 = 0; 
	m_h = 0.3;
	m_open =  FALSE;//tring by bamboo(脱离电机调试的话，改成true
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_brush.CreateSolidBrush(RGB(200,255,200));
	OutputDebugString("\n here0");
	Algorithm();

	m_LFH = 0;
	m_LFK = 0;
	m_LHH = 0;
	m_LHK = 0;
	m_RFH = 0;
	m_RFK = 0;
	m_RHH = 0;
	m_RHK = 0;
	m_pitnum = 0;
	m_rollnum = 0;
}

void CXWRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CXWRobotDlg)
	DDX_Control(pDX, IDC_SLIDER_RH, m_RH);
	DDX_Control(pDX, IDC_SLIDER_RF, m_RF);
	DDX_Control(pDX, IDC_SLIDER_LH, m_LH);
	DDX_Control(pDX, IDC_SLIDER_LF, m_LF);
	DDX_Control(pDX, IDC_BUTTON_Pause, m_Pause);
	DDX_Control(pDX, IDC_BUTTON_Continue, m_Continue);
	DDX_Control(pDX, IDC_BUTTON_Walk, m_Walk);
	DDX_Control(pDX, IDC_BUTTON_Trot, m_Trot);
	DDX_Control(pDX, IDC_BUTTON_Stop, m_Stop);
	DDX_Control(pDX, IDC_BUTTON_ModeSettings, m_ModeSettings);
	DDX_Control(pDX, IDC_BUTTON_AEnable, m_AEnable);
	DDX_Control(pDX, IDC_BUTTON_ADisable, m_ADisable);
	DDX_Control(pDX, IDC_BUTTON_OpenDevice, m_OpenDevice);
	DDX_Control(pDX, IDC_BUTTON_MoveRelative, m_MoveRelative);
	DDX_Control(pDX, IDC_BUTTON_Absolute, m_MoveAbsolute);
	DDX_Control(pDX, IDC_BUTTON_Halt, m_Halt);
	DDX_Control(pDX, IDC_BUTTON_Enable, m_Enable);
	DDX_Control(pDX, IDC_BUTTON_Disable, m_Disable);
	DDX_Control(pDX, IDC_BUTTON_DeviceSettings, m_DeviceSettings);
	DDX_Text(pDX, IDC_EDIT_AVelocity, m_AVelocity);
	DDX_Text(pDX, IDC_EDIT_DVelocity, m_DVelocity);
	DDX_Text(pDX, IDC_EDIT_NodeID, m_NodeID);
	DDX_Text(pDX, IDC_EDIT_RTPosition, m_RTPosition);
	DDX_Text(pDX, IDC_EDIT_StartPosition, m_StartPosition);
	DDX_Text(pDX, IDC_EDIT_Target, m_Target);
	DDX_Text(pDX, IDC_EDIT_Velocity, m_Velocity);
	DDX_Text(pDX, IDC_EDIT_HipBacklash, m_HipBacklash);
	DDX_Text(pDX, IDC_EDIT_KneeBacklash, m_KneeBacklash);
	DDX_Text(pDX, IDC_EDIT_Period, m_Period);
	DDV_MinMaxInt(pDX, m_Period, 1, 1000000);
	DDX_Text(pDX, IDC_EDIT_a, m_a);
	DDX_Text(pDX, IDC_EDIT_b, m_b);
	DDX_Text(pDX, IDC_EDIT_c, m_c);
	DDX_Text(pDX, IDC_EDIT_SW, m_SW);
	DDX_Text(pDX, IDC_EDIT_Amplitude, m_Amplitude);
	DDV_MinMaxDouble(pDX, m_Amplitude, 0.1, 10.);
	DDX_Text(pDX, IDC_EDIT_Hip1, m_Hip1);
	DDX_Text(pDX, IDC_EDIT_knee1, m_Knee1);
	DDX_Text(pDX, IDC_EDIT_knee2, i);
	DDX_Text(pDX, IDC_EDIT_H, m_h);
	DDV_MinMaxDouble(pDX, m_h, 0., 0.7);
	//}}AFX_DATA_MAP
	DDX_Text(pDX, IDC_EDIT_Rev, e_EditReceive);
	DDX_Control(pDX, IDC_EDIT_Send, m_EditSend);
	DDX_Control(pDX, IDC_COMBO1, m_comb1);
	DDX_Control(pDX, IDC_COMBO2, m_comb2);
	DDX_Control(pDX, IDC_MSCOMM1, m_mscom);
	DDX_Text(pDX, IDC_EDIT_LFH, m_LFH);
	DDV_MinMaxLong(pDX, m_LFH, -10000, 10000);
	DDX_Text(pDX, IDC_EDIT_LFK, m_LFK);
	DDV_MinMaxLong(pDX, m_LFK, -10000, 10000);
	DDX_Text(pDX, IDC_EDIT_LHH, m_LHH);
	DDV_MinMaxLong(pDX, m_LHH, -10000, 10000);
	DDX_Text(pDX, IDC_EDIT_LHK, m_LHK);
	DDV_MinMaxLong(pDX, m_LHK, -10000, 10000);
	DDX_Text(pDX, IDC_EDIT_RFH, m_RFH);
	DDV_MinMaxLong(pDX, m_RFH, -10000, 10000);
	DDX_Text(pDX, IDC_EDIT_RFK, m_RFK);
	DDV_MinMaxLong(pDX, m_RFK, -10000, 10000);
	//  DDX_Control(pDX, IDC_EDIT_RHH, m_RHH);
	DDX_Text(pDX, IDC_EDIT_RHH, m_RHH);
	DDV_MinMaxLong(pDX, m_RHH, -10000, 10000);
	DDX_Text(pDX, IDC_EDIT_RHK, m_RHK);
	DDV_MinMaxLong(pDX, m_RHK, -10000, 10000);
	DDX_Control(pDX, IDC_BUTTON_MoveRelative2, m_MoveRelative2);
	DDX_Control(pDX, IDC_BUTTON_Absolute2, m_MoveAbsolute2);
	DDX_Control(pDX, IDC_SLIDER_pit, m_pit);
	DDX_Control(pDX, IDC_SLIDER_roll, m_roll);
	DDX_Text(pDX, IDC_EDIT_pit, m_pitnum);
	//  DDX_Control(pDX, IDC_EDIT_roll, m_rollnum);
	DDX_Text(pDX, IDC_EDIT_roll, m_rollnum);
}

BEGIN_MESSAGE_MAP(CXWRobotDlg, CDialog)
	//{{AFX_MSG_MAP(CXWRobotDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_EN_CHANGE(IDC_EDIT_NodeID, OnChangeEDITNodeID)
	ON_EN_CHANGE(IDC_EDIT_Target, OnChangeEDITTarget)
	ON_EN_CHANGE(IDC_EDIT_Velocity, OnChangeEDITVelocity)
	ON_EN_CHANGE(IDC_EDIT_AVelocity, OnChangeEDITAVelocity)
	ON_EN_CHANGE(IDC_EDIT_DVelocity, OnChangeEDITDVelocity)
	ON_BN_CLICKED(IDC_BUTTON_OpenDevice, OnBUTTONOpenDevice)
	ON_BN_CLICKED(IDC_BUTTON_DeviceSettings, OnBUTTONDeviceSettings)
	ON_BN_CLICKED(IDC_BUTTON_Enable, OnBUTTONEnable)
	ON_BN_CLICKED(IDC_BUTTON_Disable, OnBUTTONDisable)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative, OnBUTTONMoveRelative)
	ON_BN_CLICKED(IDC_BUTTON_Halt, OnBUTTONHalt)
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON_Solo, OnBUTTONSolo)
	ON_WM_CTLCOLOR()
	ON_EN_CHANGE(IDC_EDIT_Amplitude, OnChangeEDITAmplitude)
	ON_EN_CHANGE(IDC_EDIT_HipBacklash, OnChangeEDITHipBacklash)
	ON_EN_CHANGE(IDC_EDIT_KneeBacklash, OnChangeEDITKneeBacklash)
	ON_EN_CHANGE(IDC_EDIT_Period, OnChangeEDITPeriod)
	ON_BN_CLICKED(IDC_BUTTON_ModeSettings, OnBUTTONModeSettings)
	ON_BN_CLICKED(IDC_BUTTON_ADisable, OnBUTTONADisable)
	ON_BN_CLICKED(IDC_BUTTON_AEnable, OnBUTTONAEnable)
	ON_BN_CLICKED(IDC_BUTTON_Walk, OnBUTTONWalk)
	ON_BN_CLICKED(IDC_BUTTON_Trot, OnBUTTONTrot)
	ON_BN_CLICKED(IDC_BUTTON_Stop, OnBUTTONStop)
	ON_BN_CLICKED(IDC_BUTTON_Pause, OnBUTTONPause)
	ON_BN_CLICKED(IDC_BUTTON_Continue, OnBUTTONContinue)
	ON_EN_CHANGE(IDC_EDIT_H, OnChangeEditH)
	//}}AFX_MSG_MAP
	ON_EN_CHANGE(IDC_EDIT_StartPosition, &CXWRobotDlg::OnEnChangeEditStartposition)
	ON_BN_CLICKED(IDC_BUTTON_OPEN, &CXWRobotDlg::OnBnClickedButtonOpen)
	ON_BN_CLICKED(IDC_BUTTON_SEND, &CXWRobotDlg::OnBnClickedButtonSend)
	ON_BN_CLICKED(IDC_BUTTON_CLEAN, &CXWRobotDlg::OnBnClickedButtonClean)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE, &CXWRobotDlg::OnBnClickedButtonClose)
	ON_EN_CHANGE(IDC_EDIT_RTPosition, &CXWRobotDlg::OnEnChangeEditRtposition)
	ON_BN_CLICKED(IDC_BUTTON_Remember, &CXWRobotDlg::OnBnClickedButtonRemember)
	ON_BN_CLICKED(IDC_BUTTON_Absolute, &CXWRobotDlg::OnBnClickedButtonAbsolute)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative2, &CXWRobotDlg::OnClickedButtonMoverelative2)
	ON_BN_CLICKED(IDC_BUTTON_Absolute2, &CXWRobotDlg::OnClickedButtonAbsolute2)
	ON_BN_CLICKED(IDC_BUTTON_Absolute3, &CXWRobotDlg::OnBnClickedButtonAbsolute3)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative3, &CXWRobotDlg::OnBnClickedButtonMoverelative3)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative5, &CXWRobotDlg::OnBnClickedButtonMoverelative5)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative6, &CXWRobotDlg::OnBnClickedButtonMoverelative6)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative13, &CXWRobotDlg::OnBnClickedButtonMoverelative13)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative9, &CXWRobotDlg::OnBnClickedButtonMoverelative9)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative17, &CXWRobotDlg::OnBnClickedButtonMoverelative17)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative14, &CXWRobotDlg::OnBnClickedButtonMoverelative14)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative7, &CXWRobotDlg::OnBnClickedButtonMoverelative7)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative10, &CXWRobotDlg::OnBnClickedButtonMoverelative10)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative18, &CXWRobotDlg::OnBnClickedButtonMoverelative18)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative15, &CXWRobotDlg::OnBnClickedButtonMoverelative15)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative11, &CXWRobotDlg::OnBnClickedButtonMoverelative11)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative19, &CXWRobotDlg::OnBnClickedButtonMoverelative19)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative8, &CXWRobotDlg::OnBnClickedButtonMoverelative8)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative16, &CXWRobotDlg::OnBnClickedButtonMoverelative16)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative12, &CXWRobotDlg::OnBnClickedButtonMoverelative12)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative20, &CXWRobotDlg::OnBnClickedButtonMoverelative20)
	ON_BN_CLICKED(IDC_BUTTON_MoveRelative21, &CXWRobotDlg::OnBnClickedButtonMoverelative21)
	ON_BN_CLICKED(IDC_CAM, &CXWRobotDlg::OnBnClickedCam)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CXWRobotDlg message handlers

BOOL CXWRobotDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here

	m_oUpdateActive = FALSE;
	
	m_oImmediately = TRUE;
	PauseIndex=1;
    FirstIndex=1;
	i=0;
    m_LF.SetRange((int)(-0.22*m_Amplitude*100),(int)(0.22*m_Amplitude*100),TRUE);
	m_LF.SetPos(0);
	m_RF.SetRange((int)(-0.22*m_Amplitude*100),(int)(0.22*m_Amplitude*100),TRUE);
	m_RF.SetPos(0);
	m_LH.SetRange((int)(-0.22*m_Amplitude*100),(int)(0.22*m_Amplitude*100),TRUE);
	m_LH.SetPos(0);
	m_RH.SetRange((int)(-0.22*m_Amplitude*100),(int)(0.22*m_Amplitude*100),TRUE);
	m_RH.SetPos(0);
	//角度值范围
	m_pit.SetRange((int)(-15),(int)(15),TRUE);
	m_pit.SetPos(0);
	m_roll.SetRange((int)(-15),(int)(15),TRUE);
	m_roll.SetPos(0);

 	m_ADisable.EnableWindow(false);
	m_MoveAbsolute2.EnableWindow(false);
	m_MoveRelative2.EnableWindow(false);
	m_AEnable.EnableWindow(false);
	m_Walk.EnableWindow(false);
	m_Trot.EnableWindow(false);
	m_Stop.EnableWindow(false);
    m_Pause.EnableWindow(false);
	m_Continue.EnableWindow(false);
	
	if(m_open)//OnBUTTONOpenDevice()) 这里应该有问题……除了初始化都没触发
	{
		//if(SetTimer(1,100,NULL)) m_oUpdateActive = TRUE;
		m_OpenDevice.EnableWindow(false);
		m_ModeSettings.EnableWindow(true);
	}
	else
	{
		m_Enable.EnableWindow(false);
		m_Disable.EnableWindow(false);
		m_MoveRelative.EnableWindow(false);
		m_MoveAbsolute.EnableWindow(false);
		m_Halt.EnableWindow(false);
		m_DeviceSettings.EnableWindow(false);
        
		m_ModeSettings.EnableWindow(false);
		
		m_OpenDevice.EnableWindow(true);
	}
	
    CString str;
	if(GetDlgItemText(IDC_BUTTON_Solo,str),str=="S-Joint>>")//S-Joint
	{
	
		if(m_rectLarge.IsRectEmpty())
			//IsRectEmpty：检测矩形区域是否为空。
			//如果矩形的宽和高为0或者为一个负值，则说明此矩形为空，返回非0值，否则，返回0.
		{
			CRect rectSeperator;
			GetWindowRect(&m_rectLarge);//获取窗口区域大小
			GetDlgItem(IDC_STATICSeparator)->GetWindowRect(&rectSeperator);
			//GetDlgItem(IDC_STATICSeparator)  这部分获取控件IDC_STATICSeparator的对象指针
			//整个是获取该控件的区域大小到rectSeperator中
			m_rectSmall.left=m_rectLarge.left;
			m_rectSmall.top=m_rectLarge.top;
			m_rectSmall.bottom=m_rectLarge.bottom;
			m_rectSmall.right=rectSeperator.right;
		}
		SetWindowPos(NULL,0,0,m_rectSmall.Width(),m_rectSmall.Height(),SWP_NOMOVE|SWP_NOZORDER);
		//改变窗口的位置与状态
	}
	if(1)//这是串口输入部分
	{
	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
    //  执行此操作
    SetIcon(m_hIcon, TRUE);         // 设置大图标
    SetIcon(m_hIcon, FALSE);        // 设置小图标

    // TODO: 在此添加额外的初始化代码

    // 串口选择组合框
    CString str;
    int i;
    for(i = 0; i < 12; i++)
    {
        str.Format(_T("COM %d"), i+1);
        m_comb1.InsertString(i, str);       //将COM 填入comb1
    }
    m_comb1.SetCurSel(2);                   //预置为COM 3

    //波特率选择组合框
    CString str1[]={_T("300"),_T("600"),_T("1200"),_T("2400"),_T("4800"),_T("9600"),
                    _T("19200"),_T("38400"),_T("43000"),_T("56000"),_T("57600"),_T("115200")};
					
    for(i = 0; i < 12; i++)
    {
        int judge_tf = m_comb2.AddString(str1[i]);
        if((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
            MessageBox(_T("Build baud error!"));
    }
    m_comb2.SetCurSel(0);                   //预置波特率为115200
	}
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CXWRobotDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CXWRobotDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CXWRobotDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

HBRUSH CXWRobotDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor) 
{
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);
	
	// TODO: Change any attributes of the DC here
	
	// TODO: Return a different brush if the default is not desired
   //	return hbr;
	if (pWnd->GetDlgCtrlID() == IDC_EDIT_SW)//GetDlgCtrlID()的功能是获取指定控件的ID号；
											//PWnd 包含了要求颜色的控件的指针
	{										//pDC 包含了子窗口的显示设备环境的指针
        pDC->SetTextColor(RGB(0,255,0)); //设置字体颜se
		pDC->SetBkMode(TRANSPARENT);
        return CreateSolidBrush(RGB(0,0,0));
	}
	if (pWnd->GetDlgCtrlID() == IDC_EDIT_Hip1)
	{
        pDC->SetTextColor(RGB(255,0,0)); //设置字体颜se
		pDC->SetBkMode(TRANSPARENT);
        return CreateSolidBrush(RGB(0,0,0));
	}
	if (pWnd->GetDlgCtrlID() == IDC_EDIT_knee1)
	{
        pDC->SetTextColor(RGB(255,0,0)); //设置字体颜se
		pDC->SetBkMode(TRANSPARENT);
        return CreateSolidBrush(RGB(0,0,0));
	}
	if (pWnd->GetDlgCtrlID() == IDC_STATIC)
        pDC->SetTextColor(RGB(0,0,255)); //设置字体颜se
   	return m_brush;
	/*
	if (pWnd->GetDlgCtrlID()==IDC_STATIC)     
	{     
		pDC->SetTextColor(RGB(0,255,0));       
	//	pDC->SetBkMode(TRANSPARENT);      
	//  pDC->SetBkColor(RGB(255,0,0));      
	    return m_brush;                
	}  
    //	return hbr;*/
}//设置字体颜色
void CXWRobotDlg::FormatTime(time_t time1, char *szTime)
{
    struct tm tm1;


	#ifdef WIN32
		tm1 = *localtime(&time1);
	#else
		localtime_r(&time1, &tm1 );
	#endif
		sprintf( szTime, "%4.4d%2.2d%2.2d%2.2d%2.2d%2.2d",
			   tm1.tm_year+1900, tm1.tm_mon+1, tm1.tm_mday,
				 tm1.tm_hour, tm1.tm_min,tm1.tm_sec);
}
void CXWRobotDlg::Algorithm()
{
	memcpy(SWalkLF,SWalkLF1,sizeof(SWalkLF1));
	memcpy(SWalkRF,SWalkRF1,sizeof(SWalkRF1));
	memcpy(SWalkLH,SWalkLH1,sizeof(SWalkLH1));
	memcpy(SWalkRH,SWalkRH1,sizeof(SWalkRH1));
	memcpy(WalkLF,WalkLF1,sizeof(WalkLF1));
	memcpy(WalkRF,WalkRF1,sizeof(WalkRF1));
	memcpy(WalkLH,WalkLH1,sizeof(WalkLH1));
	memcpy(WalkRH,WalkRH1,sizeof(WalkRH1));

	memcpy(SWLFK,SWLFK1,sizeof(SWLFK1));
	memcpy(SWRFK,SWRFK1,sizeof(SWRFK1));
	memcpy(SWLHK,SWLHK1,sizeof(SWLHK1));
	memcpy(SWRHK,SWRHK1,sizeof(SWRHK1));
	memcpy(WLFK,WLFK1,sizeof(WLFK1));
	memcpy(WRFK,WRFK1,sizeof(WRFK1));
	memcpy(WLHK,WLHK1,sizeof(WLHK1));
	memcpy(WRHK,WRHK1,sizeof(WRHK1));

	memcpy(STrotLF,STrotLF1,sizeof(STrotLF1));
	memcpy(STrotRF,STrotRF1,sizeof(STrotRF1));
	memcpy(STrotLH,STrotLH1,sizeof(STrotLH1));
	memcpy(STrotRH,STrotRH1,sizeof(STrotRH1));
	memcpy(TrotLF,TrotLF1,sizeof(TrotLF1));
	memcpy(TrotRF,TrotRF1,sizeof(TrotRF1));
	memcpy(TrotLH,TrotLH1,sizeof(TrotLH1));
	memcpy(TrotRH,TrotRH1,sizeof(TrotRH1));

#if 1
	char s[100]=" ";
		sprintf(s,"WalkLF[11]=%10f  SWLFK[10]=%10f;%10f  t:%3d,%3d",WalkLF[11],SWLFK[10],TrotRH[5],Stime,Ltime);
		OutputDebugString("\n 这次运行Algorithm前的测试数据");
		OutputDebugString((LPCSTR)s);
#endif
		//j相当于时间t的线性映射 

	for(j=0;j<Stime;j++)
	{
		SWalkLF[j]=m_Amplitude*SWalkLF[j];
		SWalkRF[j]=m_Amplitude*SWalkRF[j];
		SWalkLH[j]=m_Amplitude*SWalkLH[j];
		SWalkRH[j]=m_Amplitude*SWalkRH[j];
	}

	for(j=0;j<Ltime;j++)//changed
	{
		WalkLF[j]=m_Amplitude*WalkLF[j];
		WalkRF[j]=m_Amplitude*WalkRF[j];
		WalkLH[j]=m_Amplitude*WalkLH[j];
		WalkRH[j]=m_Amplitude*WalkRH[j];
	}
	for( j=0;j<20;j++)
	{
		STrotLF[j]=m_Amplitude*STrotLF[j];
		STrotRF[j]=m_Amplitude*STrotRF[j];
		STrotLH[j]=m_Amplitude*STrotLH[j];
		STrotRH[j]=m_Amplitude*STrotRH[j];
		TrotLF[j]=m_Amplitude*TrotLF[j];
		TrotRF[j]=m_Amplitude*TrotRF[j];
		TrotLH[j]=m_Amplitude*TrotLH[j];
		TrotRH[j]=m_Amplitude*TrotRH[j];
	}

//SW__K
	for(i=0;i<Stime;i++)
	{	    
		SWLFK[i]=SWLFK[i]*156*2000/(2*pi);
		SWRFK[i]=SWRFK[i]*156*2000/(2*pi);
		SWLHK[i]=SWLHK[i]*156*2000/(2*pi);
		SWRHK[i]=SWRHK[i]*156*2000/(2*pi);
	}
//W__K


	for(i=0;i<Ltime;i++)
	{
		WLFK[i]=WLFK[i]*156*2000/(2*pi);
		WRFK[i]=WRFK[i]*156*2000/(2*pi);
		WLHK[i]=WLHK[i]*156*2000/(2*pi);
		WRHK[i]=WRHK[i]*156*2000/(2*pi);
	}


///////////////////////////////////////////////////////////////////////Trot
/////////////////////////////////////////////////////////////LFK
	for( i=0; i<17;i++)
	{
		STLFK[i]=asin((330-250*sin(1.125-STrotLF[i]))/150)-0.770;
	//	STLFK[i]=STLFK[i]*156*2000/(2*pi);
		STLFK[i]=STrotLF[i]*156*2000/(2*pi);
	}
	for( i=17; i<20;i++)
	{
		STLFK[i]=asin((330-250*sin(1.125-STrotLF[i]))/150)-0.3*(m_a*(1.125-STrotLF[i])*(1.125-STrotLF[i])+m_b*(1.125-STrotLF[i])+m_c)-0.770;
		STLFK[i]=STLFK[i]*156*2000/(2*pi);
	}
//
	for( i=0; i<6;i++)
	{
		TLFK[i]=asin((330-250*sin(1.125-TrotLF[i]))/150)-0.3*(m_a*(1.125-TrotLF[i])*(1.125-TrotLF[i])+m_b*(1.125-TrotLF[i])+m_c)-0.770;
		TLFK[i]=TLFK[i]*156*2000/(2*pi);
	}
	for( i=6; i<16;i++)
	{
		TLFK[i]=asin((330-250*sin(1.125-TrotLF[i]))/150)-0.770;
//		TLFK[i]=TLFK[i]*156*2000/(2*pi);
		TLFK[i]=TrotLF[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		TLFK[i]=asin((330-250*sin(1.125-TrotLF[i]))/150)-0.3*(m_a*(1.125-TrotLF[i])*(1.125-TrotLF[i])+m_b*(1.125-TrotLF[i])+m_c)-0.770;
		TLFK[i]=TLFK[i]*156*2000/(2*pi);
	}
/////////////////////////////////////////////////////////////RFK
	for( i=0; i<8;i++)
	{
		STRFK[i]=asin((330-250*sin(1.125-STrotRF[i]))/150)-0.770;
	//	STRFK[i]=STRFK[i]*156*2000/(2*pi);
		STRFK[i]=STrotRF[i]*156*2000/(2*pi);
	}
	for( i=8; i<16;i++)
	{
		STRFK[i]=asin((330-250*sin(1.125-STrotRF[i]))/150)-0.3*(m_a*(1.125-STrotRF[i])*(1.125-STrotRF[i])+m_b*(1.125-STrotRF[i])+m_c)-0.770;
		STRFK[i]=STRFK[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		STRFK[i]=asin((330-250*sin(1.125-STrotRF[i]))/150)-0.770;
//		STRFK[i]=STRFK[i]*156*2000/(2*pi);
		STRFK[i]=STrotRF[i]*156*2000/(2*pi);
	}
//
	for( i=0; i<6;i++)
	{
		TRFK[i]=asin((330-250*sin(1.125-TrotRF[i]))/150)-0.770;
//		TRFK[i]=TRFK[i]*156*2000/(2*pi);
		TRFK[i]=TrotRF[i]*156*2000/(2*pi);
	}
	for( i=6; i<16;i++)
	{
		TRFK[i]=asin((330-250*sin(1.125-TrotRF[i]))/150)-0.3*(m_a*(1.125-TrotRF[i])*(1.125-TrotRF[i])+m_b*(1.125-TrotRF[i])+m_c)-0.770;
		TRFK[i]=TRFK[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		TRFK[i]=asin((330-250*sin(1.125-TrotRF[i]))/150)-0.770;
//		TRFK[i]=TRFK[i]*156*2000/(2*pi);
		TRFK[i]=TrotRF[i]*156*2000/(2*pi);
	}
	/////////////////////////////////////////////////////////LHK
	for( i=0; i<8;i++)
	{
		STLHK[i]=asin((330-250*sin(1.125-STrotLH[i]))/150)-0.770;
//		STLHK[i]=STLHK[i]*156*2000/(2*pi);
		STLHK[i]=STrotLH[i]*156*2000/(2*pi);
	}
	for( i=8; i<16;i++)
	{
		STLHK[i]=asin((330-250*sin(1.125-STrotLH[i]))/150)-0.3*(m_a*(1.125-STrotLH[i])*(1.125-STrotLH[i])+m_b*(1.125-STrotLH[i])+m_c)-0.770;
		STLHK[i]=STLHK[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		STLHK[i]=asin((330-250*sin(1.125-STrotLH[i]))/150)-0.770;
//		STLHK[i]=STLHK[i]*156*2000/(2*pi);
		STLHK[i]=STrotLH[i]*156*2000/(2*pi);
	}
//
	for( i=0; i<6;i++)
	{
		TLHK[i]=asin((330-250*sin(1.125-TrotLH[i]))/150)-0.770;
//		TLHK[i]=TLHK[i]*156*2000/(2*pi);
		TLHK[i]=TrotLH[i]*156*2000/(2*pi);
	}
	for( i=6; i<16;i++)
	{
		TLHK[i]=asin((330-250*sin(1.125-TrotLH[i]))/150)-0.3*(m_a*(1.125-TrotLH[i])*(1.125-TrotLH[i])+m_b*(1.125-TrotLH[i])+m_c)-0.770;
		TLHK[i]=TLHK[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		TLHK[i]=asin((330-250*sin(1.125-TrotLH[i]))/150)-0.770;
//		TLHK[i]=TLHK[i]*156*2000/(2*pi);
		TLHK[i]=TrotLH[i]*156*2000/(2*pi);
	}
///////////////////////////////////////////////////////////RHK
	for( i=0; i<16;i++)
	{
		STRHK[i]=asin((330-250*sin(1.125-STrotRH[i]))/150)-0.770;
//		STRHK[i]=STRHK[i]*156*2000/(2*pi);
		STRHK[i]=STrotRH[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		STRHK[i]=asin((330-250*sin(1.125-STrotRH[i]))/150)-0.3*(m_a*(1.125-STrotRH[i])*(1.125-STrotRH[i])+m_b*(1.125-STrotRH[i])+m_c)-0.770;
		STRHK[i]=STRHK[i]*156*2000/(2*pi);
	}
//
	for( i=0; i<6;i++)
	{
		TRHK[i]=asin((330-250*sin(1.125-TrotRH[i]))/150)-0.3*(m_a*(1.125-TrotRH[i])*(1.125-TrotRH[i])+m_b*(1.125-TrotRH[i])+m_c)-0.770;
		TRHK[i]=TRHK[i]*156*2000/(2*pi);
	}
	for( i=6; i<16;i++)
	{
		TRHK[i]=asin((330-250*sin(1.125-TrotRH[i]))/150)-0.770;
	//	TRHK[i]=TRHK[i]*156*2000/(2*pi);
		TRHK[i]=TrotRH[i]*156*2000/(2*pi);
	}
	for( i=16; i<20;i++)
	{
		TRHK[i]=asin((330-250*sin(1.125-TrotRH[i]))/150)-0.3*(m_a*(1.125-TrotRH[i])*(1.125-TrotRH[i])+m_b*(1.125-TrotRH[i])+m_c)-0.770;
		TRHK[i]=TRHK[i]*156*2000/(2*pi);
	}
	/////////////////////////////////////////////////////
	for(j=0;j<Stime;j++)
	{
		SWalkLF[j]=SWalkLF[j]*260*2000/(2*pi);
		SWalkRF[j]=SWalkRF[j]*260*2000/(2*pi);
		SWalkLH[j]=SWalkLH[j]*260*2000/(2*pi);
		SWalkRH[j]=SWalkRH[j]*260*2000/(2*pi);
	}
	for(j=0;j<Ltime;j++)
	{
		WalkLF[j]=WalkLF[j]*260*2000/(2*pi);
		WalkRF[j]=WalkRF[j]*260*2000/(2*pi);
		WalkLH[j]=WalkLH[j]*260*2000/(2*pi);
		WalkRH[j]=WalkRH[j]*260*2000/(2*pi);
	}
	for( j=0;j<20;j++)
	{
		STrotLF[j]=STrotLF[j]*260*2000/(2*pi);
		STrotRF[j]=STrotRF[j]*260*2000/(2*pi);
		STrotLH[j]=STrotLH[j]*260*2000/(2*pi);
		STrotRH[j]=STrotRH[j]*260*2000/(2*pi);
		TrotLF[j]=TrotLF[j]*260*2000/(2*pi);
		TrotRF[j]=TrotRF[j]*260*2000/(2*pi);
		TrotLH[j]=TrotLH[j]*260*2000/(2*pi);
		TrotRH[j]=TrotRH[j]*260*2000/(2*pi);
	}
#if 1
		OutputDebugString("\n here1");
		
		sprintf(s,"%10f,WalkLF[11]=%10f;%10f",SWLFK[10],WalkLF[11],TrotRH[5]);
		OutputDebugString("\n 这次运行Algorithm后的测试数据SWLFK[10]=");
		OutputDebugString((LPCSTR)s);
#endif
}

void CXWRobotDlg::OnChangeEDITNodeID() //S-joint NO.
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
	if(m_NodeID >= 1 && m_NodeID <= 9)
	{
		if(SetTimer(1,100,NULL))
		m_oUpdateActive = TRUE;
		m_RTPosition = 0;
		m_StartPosition = 0;
	}
	else
	{
		AfxMessageBox("Node ID from 1 to 9!",MB_ICONINFORMATION);
	}

	
}

void CXWRobotDlg::OnChangeEDITTarget() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnChangeEDITVelocity() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnChangeEDITAVelocity() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnChangeEDITDVelocity() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnBUTTONOpenDevice() 
{
	// TODO: Add your control notification handler code here
	HANDLE hNewKeyHandle;

	//OpenDevice
	hNewKeyHandle = VCS_OpenDeviceDlg(&m_dErrorCode);//VCS_OpenDeviceDlg：这是maxon提供的函数
	if(hNewKeyHandle)
	{
		//Close Previous Device
		if(m_KeyHandle) VCS_CloseDevice(m_KeyHandle,&m_dErrorCode);
		m_KeyHandle = hNewKeyHandle;
	//	if(SetTimer(1,100,NULL)) m_oUpdateActive = TRUE;
		m_open = TRUE;
		CXWRobotDlg::OnInitDialog();
		return; //TRUE;
	}
	else
	{
		AfxMessageBox("Can't open device!",MB_ICONINFORMATION);
		KillTimer(1);
	    m_oUpdateActive = FALSE;
		m_open = FALSE;
		CXWRobotDlg::OnInitDialog();
		return ;// FALSE;
	}
	
}//激活maxon电机

void CXWRobotDlg::OnBUTTONDeviceSettings() 
{
	// TODO: Add your control notification handler code here
	if(VCS_ClearFault(m_KeyHandle,m_NodeID,&m_dErrorCode))
	{
	    //Write Profile Position Mode
		if(VCS_SetOperationMode(m_KeyHandle,m_NodeID,0x01/*Profile Position Mode*/,&m_dErrorCode))
		{
			//Write Profile Position Objects
			if(VCS_SetPositionProfile(m_KeyHandle,m_NodeID,m_Velocity,m_AVelocity,m_DVelocity,&m_dErrorCode))
			{
				//Read Actual Position
				if(VCS_GetPositionIs(m_KeyHandle,m_NodeID,&m_StartPosition,&m_dErrorCode))
				{
					return;// TRUE;
				}
			}
		}
	}
	ShowErrorInformation(m_dErrorCode);
 	return;// FALSE;
}

void CXWRobotDlg::OnBUTTONEnable() 
{
	// TODO: Add your control notification handler code here
	BOOL oFault = FALSE;

	if(!VCS_GetFaultState(m_KeyHandle,m_NodeID,&oFault,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
		return;
	}

	if(oFault)
	{
		if(!VCS_ClearFault(m_KeyHandle,m_NodeID,&m_dErrorCode))
		{
			ShowErrorInformation(m_dErrorCode);
			return;
		}
	}

	if(!VCS_SetEnableState(m_KeyHandle,m_NodeID,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

void CXWRobotDlg::OnBUTTONDisable() 
{
	// TODO: Add your control notification handler code here
	if(!VCS_SetDisableState(m_KeyHandle,m_NodeID,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

void CXWRobotDlg::OnBUTTONMoveRelative() 
{
	// TODO: Add your control notification handler code here
	if(VCS_GetPositionIs(m_KeyHandle,m_NodeID,&m_StartPosition,&m_dErrorCode))
	{
		if(!VCS_MoveToPosition(m_KeyHandle,m_NodeID,m_Target,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
		{
			ShowErrorInformation(m_dErrorCode);
		}
	}

	UpdateStatus();
}

void CXWRobotDlg::OnBUTTONHalt() 
{
	// TODO: Add your control notification handler code here
	if(!VCS_HaltPositionMovement(m_KeyHandle,m_NodeID,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

BOOL CXWRobotDlg::ShowErrorInformation(DWORD dErrorCode)
{
	char* pStrErrorInfo;
	CString strDescription;

	if((pStrErrorInfo = (char*)malloc(100)) == NULL)
	{
		MessageBox("Not enough memory to allocate buffer for error information string\n","System Error");

		return FALSE;
	}

	if(VCS_GetErrorInfo(dErrorCode,pStrErrorInfo,100))
	{
		strDescription = pStrErrorInfo;
		AfxMessageBox(strDescription,MB_ICONINFORMATION);

		free(pStrErrorInfo);

		return TRUE;
	}
	else
	{
		free(pStrErrorInfo);
		AfxMessageBox("Error information can't be read!",MB_ICONINFORMATION);

		return FALSE;
	}
}

void CXWRobotDlg::OnTimer(UINT nIDEvent) 
{

	// TODO: Add your message handler code here and/or call default

	switch(nIDEvent)
	{
	    case 1:
		{
			if(m_oUpdateActive && !UpdateStatus())
			{
		        KillTimer(1);
	            m_oUpdateActive = FALSE;				
			}
			break;
		}
		case 2:
		{
			
			if(PauseIndex)
			{
				
				if(FirstIndex)
				{					
					m_LF.SetPos((int)(SWalkLF[i]*pi/1560));
					m_RF.SetPos((int)(SWalkRF[i]*pi/1560));
					m_LH.SetPos((int)(SWalkLH[i]*pi/1560));
					m_RH.SetPos((int)(SWalkRH[i]*pi/1560));
					if(stop_time<60)
						{	stop_time++;}
					else
						{
					# if ver==0
						if(i==0 && adj<=adj_turn)//准备抬RH:要求p>200,r<-400
						{
							if(m_pitnum<400 || m_rollnum>-400)
							{	
								if(m_pitnum<400)
								{
									RFH_adj+=200;//短
									RFK_adj+=1000;
									LFH_adj+=200;//短
									LFK_adj+=1000;
								}
								else if(m_rollnum>-400)
								{
									LFH_adj+=500;//短
									LFK_adj+=1500;
									RHH_adj-=500;//长
									RHK_adj-=1500;
									RFH_adj-=200;//长
									RFK_adj-=1000;
								}
								
								adj_flag=2;
								adjed_flag++;//调整过的次数
							}
							else
							{
								//LFH_adj=0;
								//LFK_adj=3*LFH_adj;
								//RHH_adj=0;
								//RHK_adj=3*RHH_adj;  //先试试调整后不变
								adj_flag++;//达到目的的维持次数
							}
						adj++;//调整次数
						}
						if(i==21 && adj<=adj_turn)//准备抬RF:要求p<-200,r<-400
						{
							if(adj_flag<2){
								//之前设定adj_flag=-5，所以过5次调整
								//调整完之后暂停2下
								/*LFH_adj-=500;//短-
								LFK_adj=3*LFH_adj;
								RHH_adj+=500;//长-
								RHK_adj=3*RHH_adj;*/
								if(adj_flag<0){
								 LFH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LFH_adjed;
								 LFK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LFK_adjed;
								 RFH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RFH_adjed;
								 RFK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RFK_adjed;
								 LHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHH_adjed;
								 LHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHK_adjed;
								 RHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHH_adjed;
								 RHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHK_adjed;
								}
								else
								{//保证还原……
								 LFH_adj=0;
								 LFK_adj=0;
								 RFH_adj=0;
								 RFK_adj=0;
								 LHH_adj=0;
								 LHK_adj=0;
								 RHH_adj=0;
								 RHK_adj=0;
								}
								adj_flag++;
								//这个部分可以加上反馈，变成维持水平（于地面.
							}
							else if(m_pitnum>-200 || m_rollnum>-400)//准备抬RF:要求p<-200,r<-400
							{	
								if(m_pitnum>-200)
								{
									LHH_adj+=200;//短
									LHK_adj+=1000;
									RHH_adj+=200;//短
									RHK_adj+=1000;
								}
								else if(m_rollnum>-400)
								{
								LHH_adj+=500;//短
								LHK_adj+=1500;
								RFH_adj-=0;//
								//RFH_adj-=500;//长
								RFK_adj-=1500;//
								//RFK_adj=3*RFH_adj;
								RHH_adj-=200;//
								RHK_adj-=1000;//
								}
								
								adj_flag=2;
								adjed_flag++;//调整过的次数
							}
							else
							{
								adj_flag++;//达到目的的维持次数
							}
						adj++;//调整次数
						}

					# endif

				    # if ver==0
					if(!VCS_MoveToPosition(m_KeyHandle,1,((long)SWalkLF[i]+LFH_adj+LFHS)*1,1,m_oImmediately,&m_dErrorCode))//中间的1表示“绝对”运动    //-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			//	}//not tryed yet	
					if(!VCS_MoveToPosition(m_KeyHandle,3,((long)SWalkRF[i]*(1)+RFH_adj+RFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
		     		if(!VCS_MoveToPosition(m_KeyHandle,7,((long)SWalkLH[i]*(-1)-LHH_adj+LHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			        if(!VCS_MoveToPosition(m_KeyHandle,5,((long)SWalkRH[i]*(-1)-RHH_adj+RHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}

					if(!VCS_MoveToPosition(m_KeyHandle,2,((long)SWLFK[i]+LFK_adj+LFKS)*(1),1,m_oImmediately,&m_dErrorCode))//-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}			
			    	if(!VCS_MoveToPosition(m_KeyHandle,4,((long)SWRFK[i]*(-1)-RFK_adj+RFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,8,((long)SWLHK[i]*(1)+LHK_adj+LHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,6,((long)SWRHK[i]*(-1)-RHK_adj+RHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					# else
					if(!VCS_MoveToPosition(m_KeyHandle,1,((long)SWalkLF[i]+LFHS)*1,1,m_oImmediately,&m_dErrorCode))//中间的1表示“绝对”运动    //-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			//	}//not tryed yet	
					if(!VCS_MoveToPosition(m_KeyHandle,3,((long)SWalkRF[i]*(1)+RFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
		     		if(!VCS_MoveToPosition(m_KeyHandle,7,((long)SWalkLH[i]*(-1)+LHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			        if(!VCS_MoveToPosition(m_KeyHandle,5,((long)SWalkRH[i]*(-1)+RHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}

					if(!VCS_MoveToPosition(m_KeyHandle,2,((long)SWLFK[i]+LFKS)*(1),1,m_oImmediately,&m_dErrorCode))//-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}			
			    	if(!VCS_MoveToPosition(m_KeyHandle,4,((long)SWRFK[i]*(-1)+RFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,8,((long)SWLHK[i]*(1)+LHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,6,((long)SWRHK[i]*(-1)+RHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					# endif
# if ver==0
if(i==21 || i==0 )//|| i==(Stime-1))//这里改了，主意看看对不对 时间节点也需要检查
{
	char s[130]=" ";
	sprintf(s,"i=%3d;adj_flag=%6d;adjed_flag=%6d if_1;m_pitnum=%3d;m_rollnum=%3d",i,adj_flag,(adjed_flag),m_pitnum,m_rollnum);
	OutputDebugString("\n ");
	OutputDebugString((LPCSTR)s);
if(adj_flag>(3+2))//完成稳定再动
{
	adj=0;//调整次数归零
	//adj_flag=-adjed_flag;//成功次数归零//同时用来还原
	adjed_flag=0;

 LFH_adjed=LFH_adj;
 LFK_adjed=LFK_adj;
 RFH_adjed=RFH_adj;
 RFK_adjed=RFK_adj;
 LHH_adjed=LHH_adj;
 LHK_adjed=LHK_adj;
 RHH_adjed=RHH_adj;
 RHK_adjed=RHK_adj;
 adj_flag=-ADJJ;//改成固定的来恢复……
# endif
		i++;
# if ver==0
}
}
else 
{i++;}
# endif
}
					if(i==Stime)
					{
						FirstIndex=0;
						i=0;
					}
					m_Hip1=(long)SWalkLF[i];
					m_Knee1=(long)SWLFK[i];
					UpdateData(false);
				}
				else//这里是循环形态
				{	
					m_LF.SetPos((int)(WalkLF[i]*pi/1560));
					m_RF.SetPos((int)(WalkRF[i]*pi/1560));
					m_LH.SetPos((int)(WalkLH[i]*pi/1560));
					m_RH.SetPos((int)(WalkRH[i]*pi/1560));
# if ver==0
						if(i==0 && adj<=adj_turn)//准备抬LH:要求p>400,r>500
						{
							if(adj_flag<2){
								//之前设定adj_flag=-5，所以过5次调整
								//调整完之后暂停2下
								/*LFH_adj-=500;//短-
								LFK_adj=3*LFH_adj;
								RHH_adj+=500;//长-
								RHK_adj=3*RHH_adj;*/
								if(adj_flag<0){
								 LFH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LFH_adjed;
								 LFK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LFK_adjed;
								 RFH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RFH_adjed;
								 RFK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RFK_adjed;
								 LHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHH_adjed;
								 LHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHK_adjed;
								 RHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHH_adjed;
								 RHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHK_adjed;
								}
								else
								{//保证还原……
								 LFH_adj=0;
								 LFK_adj=0;
								 RFH_adj=0;
								 RFK_adj=0;
								 LHH_adj=0;
								 LHK_adj=0;
								 RHH_adj=0;
								 RHK_adj=0;
								}
								adj_flag++;
								//这个部分可以加上反馈，变成维持水平（于地面.
							}
							else if(m_pitnum<500 || m_rollnum<600)//准备抬LH:要求p>400,r>500
							{	
								if(m_pitnum<500)
								{
									RFH_adj+=200;//短
									RFK_adj+=1000;
									LFH_adj+=200;//短
									LFK_adj+=1000;
								}
								else if(m_rollnum<600)
								{
								RFH_adj+=0;//RF靠前，Hip接近极限
								//RFH_adj+=500;//短
								RFK_adj+=1200;
								//RFK_adj=3*RFH_adj;
								LHH_adj-=800;//长
								LHK_adj-=800;//靠后，动hip往前推
								//LHK_adj=3*LHH_adj;
								LFH_adj+=700;//长500
								LFK_adj+=300;//100
								}
								
								adj_flag=2;
								adjed_flag++;//调整过的次数
							}
							else
							{
								adj_flag++;//达到目的的维持次数
							}
						adj++;//调整次数
						}
						if(i==21 && adj<=adj_turn)//准备抬LF:要求p<-300,r>450
						{
							if(adj_flag<2){
								if(adj_flag<0){
									if(adj_flag<=-(ADJJ/3)){
									 LFH_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*LFH_adjed;
									 LFK_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*LFK_adjed;
									 //这里LF先伸长了
									 RFH_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*RFH_adjed;//long+
									 RFK_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*RFK_adjed;
									}									 
								
								//LHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHH_adjed;//none
								//LHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHK_adjed;//none
								 RHH_adj+=5000/ADJJ;//none
								 RHK_adj+=14000/ADJJ;//none
								 
								//RHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHH_adjed;//short-
								 //RHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHK_adjed;
								
								 // RHH_adj=0;
								// RHK_adj=0;
								}
								else
								{//保证还原……
								 LFH_adj=0;
								 LFK_adj=0;
								 RFH_adj=0;
								 RFK_adj=0;
								// LHH_adj=0;
								// LHK_adj=0;
								// RHH_adj=0;
								// RHK_adj=0;
								}
								adj_flag++;
								//这个部分可以加上反馈，变成维持水平（于地面.
							}
							else if(m_pitnum>-200 || m_rollnum<400)//准备抬LF:要求p<-300,r>450
							{	
								if(m_pitnum>-200)//.20170317这次试验....根本就没进来过
								{
									LHH_adj+=100;//短//20170317
									LHK_adj+=500;
									RHH_adj+=200;//短
									RHK_adj+=1000;
								}
								else if(m_rollnum<400)
								{
								RHH_adj+=500;//短
								RHK_adj+=1500;//3*RHH_adj;//位置靠中，可以直接收？
								LFH_adj-=0;//LFH_adj-=500;//长(看实验结果决定改不改
								//这里动knee方便往后按，抬前腿
								LFK_adj-=1200;//LFK_adj=3*LFH_adj;
								LHH_adj-=200;//LFH_adj-=500;//长(看实验结果决定改不改
								LHK_adj-=600;//LFK_adj=3*LFH_adj;
								}
								

								
								adj_flag=2;
								adjed_flag++;//调整过的次数
							}
							else
							{
								adj_flag++;//达到目的的维持次数
							}
						adj++;//调整次数
						}
						if(i==41 && adj<=adj_turn)//准备抬RH:要求p>200,r<-450
						{
							if(adj_flag<2){
								//之前设定adj_flag=-5，所以过5次调整
								//调整完之后暂停2下
								/*LFH_adj-=500;//短-
								LFK_adj=3*LFH_adj;
								RHH_adj+=500;//长-
								RHK_adj=3*RHH_adj;*/
								if(adj_flag<0){
								 LFH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LFH_adjed;
								 LFK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LFK_adjed;
								 RFH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RFH_adjed;
								 RFK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RFK_adjed;
								 LHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHH_adjed;
								 LHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHK_adjed;
								 RHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHH_adjed;
								 RHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHK_adjed;
								}
								else
								{//保证还原……
								 LFH_adj=0;
								 LFK_adj=0;
								 RFH_adj=0;
								 RFK_adj=0;
								 LHH_adj=0;
								 LHK_adj=0;
								 RHH_adj=0;
								 RHK_adj=0;
								}
								adj_flag++;
								//这个部分可以加上反馈，变成维持水平（于地面.
							}
							else if( m_pitnum<400 ||m_rollnum>-600)//准备抬RH:要求p>200,r<-450//(m_pitnum<10 || m_rollnum>-200)
							{	
								if(m_pitnum<400)
								{
									RFH_adj+=200;//短
									RFK_adj+=1000;
									LFH_adj+=200;//短
									LFK_adj+=1000;
								}
								else if(m_rollnum>-600)//可以试试改这个？
								{
								LFH_adj+=0;
								//LFH_adj+=500;//短//这条腿刚踩下来，很靠前，hip接近极限
								LFK_adj+=800;//20170317//可以试试改这个？
								//LFK_adj=3*LFH_adj;
								RHH_adj-=1000;//长//单动hip往前顶
								RHK_adj-=400;//RHK_adj=3*RHH_adj;
								RFH_adj+=500;//duan//单动hip往前顶
								RFK_adj+=100;//RHK_adj=3*RHH_adj;
								}
								

								
								adj_flag=2;
								adjed_flag++;//调整过的次数
							}
							else
							{
								adj_flag++;//达到目的的维持次数
							}
						adj++;//调整次数
						}
						if(i==61 && adj<=adj_turn)//准备抬RF:要求p<-200,r<-400
						{
							if(adj_flag<2){

			//!!!!!	//这个地方的调整莫名的不稳！20170315
								if(adj_flag<0){
									if(adj_flag<=-(ADJJ/3)){
									 LFH_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*LFH_adjed;
									 LFK_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*LFK_adjed;
									 //这里LF先伸长了
									 RFH_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*RFH_adjed;//long+
									 RFK_adj=-(long)(adj_flag+ADJJ/3)/((float)ADJJ*2/3+1.0)*RFK_adjed;
									}									 
								
								//LHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHH_adjed;//none
								//LHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*LHK_adjed;//none
								 LHH_adj+=5000/ADJJ;//none
								 LHK_adj+=14000/ADJJ;//none
								 
								//RHH_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHH_adjed;//short-
								 //RHK_adj=-(long)adj_flag/((float)ADJJ+1.0)*RHK_adjed;
								//不还原RH
								 // RHH_adj=0;
								// RHK_adj=0;
								}
								else
								{//保证还原……
								 LFH_adj=0;
								 LFK_adj=0;
								 RFH_adj=0;
								 RFK_adj=0;
								// LHH_adj=0;
								// LHK_adj=0;
								 //RHH_adj=0;
								 //RHK_adj=0;
								}
								adj_flag++;
								//这个部分可以加上反馈，变成维持水平（于地面.
							}
							else if(m_pitnum>-300 || m_rollnum>-400)//准备抬RF:要求p<-200,r<-400
							{	
								if(m_pitnum>-300)//.20170317这次试验....根本就没进来过
								{
									LHH_adj+=500;//短
									LHK_adj+=1200;
									RHH_adj+=500;//短//20170317
									RHK_adj+=1000;//20170317

									//RFH_adj+=500;//20170317
									//RFK_adj+=800;//20170317
									//LFH_adj-=500;//chang//20170317
									//LFK_adj-=800;//20170317
								}
								else if(m_rollnum>-400)
								{
								LHH_adj+=500;//短
								LHK_adj+=1500;
								//LHK_adj=3*LHH_adj;
								RFH_adj-=0;//RFH_adj-=500;//长
								RFK_adj-=1200;//RFK_adj=3*RFH_adj;
								RHH_adj-=200;//RFH_adj-=500;//长
								RHK_adj-=600;//RFK_adj=3*RFH_adj;
								}
								
								
								adj_flag=2;
								adjed_flag++;//调整过的次数
							}
							else
							{
								adj_flag++;//达到目的的维持次数
							}
						adj++;//调整次数
						}

# endif
				    # if ver==0
					if(!VCS_MoveToPosition(m_KeyHandle,1,((long)WalkLF[i]+LFH_adj+LFHS)*1,1,m_oImmediately,&m_dErrorCode))//中间的1表示“绝对”运动    //-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			//	}//not tryed yet	
					if(!VCS_MoveToPosition(m_KeyHandle,3,((long)WalkRF[i]*(1)+RFH_adj+RFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
		     		if(!VCS_MoveToPosition(m_KeyHandle,7,((long)WalkLH[i]*(-1)-LHH_adj+LHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			        if(!VCS_MoveToPosition(m_KeyHandle,5,((long)WalkRH[i]*(-1)-RHH_adj+RHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}

					if(!VCS_MoveToPosition(m_KeyHandle,2,((long)WLFK[i]+LFK_adj+LFKS)*(1),1,m_oImmediately,&m_dErrorCode))//-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}			
			    	if(!VCS_MoveToPosition(m_KeyHandle,4,((long)WRFK[i]*(-1)-RFK_adj+RFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,8,((long)WLHK[i]*(1)+LHK_adj+LHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,6,((long)WRHK[i]*(-1)-RHK_adj+RHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					# else
					if(!VCS_MoveToPosition(m_KeyHandle,1,((long)WalkLF[i]+LFHS)*1,1,m_oImmediately,&m_dErrorCode))//中间的1表示“绝对”运动    //-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			//	}//not tryed yet	
					if(!VCS_MoveToPosition(m_KeyHandle,3,((long)WalkRF[i]*(1)+RFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
		     		if(!VCS_MoveToPosition(m_KeyHandle,7,((long)WalkLH[i]*(-1)+LHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
			        if(!VCS_MoveToPosition(m_KeyHandle,5,((long)WalkRH[i]*(-1)+RHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}

					if(!VCS_MoveToPosition(m_KeyHandle,2,((long)WLFK[i]+LFKS)*(1),1,m_oImmediately,&m_dErrorCode))//-1
					{
			            ShowErrorInformation(m_dErrorCode);
					}			
			    	if(!VCS_MoveToPosition(m_KeyHandle,4,((long)WRFK[i]*(-1)+RFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,8,((long)WLHK[i]*(1)+LHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,6,((long)WRHK[i]*(-1)+RHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					# endif
# if ver==0
if(i==0 || i==21 ||i==41 || i==61 )// || i==(Ltime-1))
{
	char s[130]=" ";
	sprintf(s,"i=%3d;adj_flag=%6d;adjed_flag=%6d in if 2;m_pitnum=%3d;m_rollnum=%3d",i,adj_flag,(adjed_flag),m_pitnum,m_rollnum);
	OutputDebugString("\n ");
	OutputDebugString((LPCSTR)s);
if(adj_flag>(3+2))//完成稳定再动
{

	adj=0;//调整次数归零
	//adj_flag=-adjed_flag;//成功次数归零//同时用来还原
	adjed_flag=0;

 LFH_adjed=LFH_adj;
 LFK_adjed=LFK_adj;
 RFH_adjed=RFH_adj;
 RFK_adjed=RFK_adj;
 LHH_adjed=LHH_adj;
 LHK_adjed=LHK_adj;
 RHH_adjed=RHH_adj;
 RHK_adjed=RHK_adj;
 adj_flag=-ADJJ;//改成固定的来恢复……
# endif
		i++;
# if ver==0
}
}
else 
{i++;}
# endif
					if(i==Ltime)//这里要改！！
					{
						i=0;
						//KillTimer(2);
						loopt++;
						if(loopt>=looptmax){ KillTimer(2);}
					}
					m_Hip1=(long)WalkLF[i];
					m_Knee1=(long)WLFK[i];
					UpdateData(false);
				}
				
			}
			break;
		}
		case 3:
		{
			if(PauseIndex)
			{			
				if(FirstIndex)
				{
					m_LF.SetPos((int)(STrotLF[i]*pi/1560));
					m_RF.SetPos((int)(STrotRF[i]*pi/1560));
					m_LH.SetPos((int)(STrotLH[i]*pi/1560));
					m_RH.SetPos((int)(STrotRH[i]*pi/1560));
					
					char s[16]=" ";
					sprintf(s,"%6d;%6d",((long)STrotLF[i]+LFHS),((long)STLFK[i]*(-1)+LFKS));
					OutputDebugString("\n STrotLF的输出数据=");
					OutputDebugString((LPCSTR)s);
				if(1)
					{
					if(((long)STrotLF[i]+LFHS)*1<50000 && ((long)STrotLF[i]+LFHS)*1>-50000)
					{
						if(!VCS_MoveToPosition(m_KeyHandle,1,((long)STrotLF[i]+LFHS),1,m_oImmediately,&m_dErrorCode))
						{
							ShowErrorInformation(m_dErrorCode);
						}
					}
					else
					{
						sprintf(s,"%10d",((long)STrotLF[i]));
						OutputDebugString("\n STrotLF[i]=");
						OutputDebugString((LPCSTR)s);
						KillTimer(3);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,3,((long)STrotRF[i]+RFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,7,((long)STrotLH[i]*(-1)+LHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,5,((long)STrotRH[i]*(-1)+RHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}

					if(!VCS_MoveToPosition(m_KeyHandle,2,((long)STLFK[i]*(-1)+LFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}					
					if(!VCS_MoveToPosition(m_KeyHandle,4,((long)STRFK[i]+RFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,8,((long)STLHK[i]*(-1)+LHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,6,((long)STRHK[i]+RHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					}
					i++;
					if(i==20)
					{
					//i=19;//暂时别动了你，先看数值对不对
						FirstIndex=0;
						i=0;
					}
					m_Hip1=(long)STrotLF[i];
					m_Knee1=(long)STLFK[i];
					UpdateData(false);
				}
				else
				{	
					m_LF.SetPos((int)(TrotLF[i]*pi/1560));
					m_RF.SetPos((int)(TrotRF[i]*pi/1560));
					m_LH.SetPos((int)(TrotLH[i]*pi/1560));
					m_RH.SetPos((int)(TrotRH[i]*pi/1560));
					
					char s[15]=" ";
					sprintf(s,"%6d;%6d",((long)TrotLF[i]+LFHS),((long)TLFK[i]*(-1)+LFKS));
					OutputDebugString("\n TrotLF的输出数据=");
					OutputDebugString((LPCSTR)s);
				if(1)
				{
				    if(!VCS_MoveToPosition(m_KeyHandle,1,((long)TrotLF[i]+LFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					
					if(!VCS_MoveToPosition(m_KeyHandle,3,((long)TrotRF[i]+RFHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,7,((long)TrotLH[i]*(-1)+LHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,5,((long)TrotRH[i]*(-1)+RHHS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}

					if(!VCS_MoveToPosition(m_KeyHandle,2,((long)TLFK[i]*(-1)+LFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}					
					if(!VCS_MoveToPosition(m_KeyHandle,4,((long)TRFK[i]+RFKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,8,((long)TLHK[i]*(-1)+LHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
					if(!VCS_MoveToPosition(m_KeyHandle,6,((long)TRHK[i]+RHKS),1,m_oImmediately,&m_dErrorCode))
					{
			            ShowErrorInformation(m_dErrorCode);
					}
				}
					i++;
					if(i==20)
					{
						i=0;
					}
					m_Hip1=(long)TrotLF[i];
					m_Knee1=(long)TLFK[i];
					UpdateData(false);
				}
			}
			break;
		}
		default: break;
	}
	CDialog::OnTimer(nIDEvent);
}

BOOL CXWRobotDlg::UpdateStatus()
{
	BOOL oEnable = TRUE;
	BOOL oResult = m_oUpdateActive;
	if(oResult)
	{
		oResult = VCS_GetEnableState(m_KeyHandle,m_NodeID,&oEnable,&m_dErrorCode);

		if(oResult)
		{
			m_OpenDevice.EnableWindow(false);
			m_DeviceSettings.EnableWindow(!oEnable);
			m_Enable.EnableWindow(!oEnable);

			

			m_Disable.EnableWindow(oEnable);
			m_MoveRelative.EnableWindow(oEnable);
			m_MoveAbsolute.EnableWindow(oEnable);
			m_Halt.EnableWindow(oEnable);

		
			
		}
		else
		{
			KillTimer(1);
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);

            m_OpenDevice.EnableWindow(false);
			m_DeviceSettings.EnableWindow(!oEnable);
			m_Enable.EnableWindow(oEnable);
			m_Disable.EnableWindow(!oEnable);
			m_MoveRelative.EnableWindow(!oEnable);
			m_MoveAbsolute.EnableWindow(!oEnable);
			m_Halt.EnableWindow(!oEnable);

			
		}
	}
	else
	{
		m_OpenDevice.EnableWindow(oEnable);
		m_DeviceSettings.EnableWindow(!oEnable);
		m_Enable.EnableWindow(!oEnable);
		m_Disable.EnableWindow(!oEnable);
		m_MoveRelative.EnableWindow(!oEnable);
		m_MoveAbsolute.EnableWindow(!oEnable);
		m_Halt.EnableWindow(!oEnable);
	}

	if(oResult)//这里明显多了出来……可以并到前面
	{
		oResult = VCS_GetPositionIs(m_KeyHandle,m_NodeID,&m_RTPosition,&m_dErrorCode);
		if(!oResult)
		{
			KillTimer(1);
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);

			m_RTPosition = 0;
			m_StartPosition = 0;
		}
	}
	else
	{
		m_RTPosition = 0;
		m_StartPosition = 0;
	}

	if(m_hWnd) UpdateData(false);

	return oResult;
}

void CXWRobotDlg::OnDestroy() 
{
	CDialog::OnDestroy();
	
	// TODO: Add your message handler code here
	//Stop Updating
	KillTimer(1);
	KillTimer(2);
	KillTimer(3);
	//Close Device
	/*
	if(VCS_SetDisableState(m_KeyHandle,1,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,2,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,3,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,4,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,5,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,6,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,7,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,8,&m_dErrorCode))
	{
	
	}
	else
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(m_KeyHandle) VCS_CloseDevice(m_KeyHandle,&m_dErrorCode);
	*/
}

void CXWRobotDlg::OnBUTTONSolo() 
{
	// TODO: Add your control notification handler code here
	CString str;
	if(GetDlgItemText(IDC_BUTTON_Solo,str),str=="S-Joint<<")
	{
		SetDlgItemText(IDC_BUTTON_Solo,"S-Joint>>");
		KillTimer(1);
        m_oUpdateActive = FALSE;
	}
	else
	{
		SetDlgItemText(IDC_BUTTON_Solo,"S-Joint<<");
		if(SetTimer(1,100,NULL)) m_oUpdateActive = TRUE;
	}

	if(m_rectLarge.IsRectEmpty())
	{
		CRect rectSeperator;
		GetWindowRect(&m_rectLarge);
		GetDlgItem(IDC_STATICSeparator)->GetWindowRect(&rectSeperator);
		m_rectSmall.left=m_rectLarge.left;
		m_rectSmall.top=m_rectLarge.top;
		m_rectSmall.bottom=m_rectLarge.bottom;
		m_rectSmall.right=m_rectLarge.right;
	}
	if(str=="S-Joint<<")
	{
		SetWindowPos(NULL,0,0,m_rectSmall.Width(),m_rectSmall.Height(),SWP_NOMOVE|SWP_NOZORDER);	
	}
	else
	{
		SetWindowPos(NULL,0,0,m_rectLarge.Width(),m_rectLarge.Height(),SWP_NOMOVE|SWP_NOZORDER);	
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CXWRobotDlg::OnChangeEDITAmplitude() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
	OutputDebugString("\n here2");
	Algorithm();
		
}

void CXWRobotDlg::OnChangeEDITHipBacklash() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnChangeEDITKneeBacklash() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnChangeEDITPeriod() 
{
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
}

void CXWRobotDlg::OnBUTTONModeSettings() 
{
	// TODO: Add your control notification handler code here
    UpdateData(true);
	if(VCS_ClearFault(m_KeyHandle,1,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,2,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,3,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,4,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,5,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,6,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,7,&m_dErrorCode)&&
		VCS_ClearFault(m_KeyHandle,8,&m_dErrorCode))
	{
	    //Write Profile Position Mode
		if(VCS_SetOperationMode(m_KeyHandle,1,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,2,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,3,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,4,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,5,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,6,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,7,0x01/*Profile Position Mode*/,&m_dErrorCode)&&
			VCS_SetOperationMode(m_KeyHandle,8,0x01/*Profile Position Mode*/,&m_dErrorCode))
		{
			//Write Profile Position Objects
			if(VCS_SetPositionProfile(m_KeyHandle,1, 7000,5000,5000,&m_dErrorCode)&&
				VCS_SetPositionProfile(m_KeyHandle,2,7000,5000,5000,&m_dErrorCode)&&
				VCS_SetPositionProfile(m_KeyHandle,3,7000,5000,5000,&m_dErrorCode)&&////
				VCS_SetPositionProfile(m_KeyHandle,4,7000,5000,5000,&m_dErrorCode)&&////这里是为了避免报错
				VCS_SetPositionProfile(m_KeyHandle,5,7000,5000,5000,&m_dErrorCode)&&
				VCS_SetPositionProfile(m_KeyHandle,6,7000,5000,5000,&m_dErrorCode)&&
				VCS_SetPositionProfile(m_KeyHandle,7,7000,5000,5000,&m_dErrorCode)&&
				VCS_SetPositionProfile(m_KeyHandle,8,7000,5000,5000,&m_dErrorCode))
			{
                m_AEnable.EnableWindow(true);
			}
			else
			{
				ShowErrorInformation(m_dErrorCode);
			}
		}
		else
		{
			ShowErrorInformation(m_dErrorCode);
		}

	}
	else
	{
	    ShowErrorInformation(m_dErrorCode);
	}

	
	
}

void CXWRobotDlg::OnBUTTONADisable() 
{
	// TODO: Add your control notification handler code here
	if(VCS_SetDisableState(m_KeyHandle,1,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,2,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,3,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,4,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,5,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,6,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,7,&m_dErrorCode)&&
		VCS_SetDisableState(m_KeyHandle,8,&m_dErrorCode))
	{
		m_AEnable.EnableWindow(true);
		m_Walk.EnableWindow(false);
		m_Trot.EnableWindow(false);
		m_ModeSettings.EnableWindow(true);
		m_ADisable.EnableWindow(false);
		m_MoveAbsolute2.EnableWindow(false);
		m_MoveRelative2.EnableWindow(false);
	}
	else
	{
		ShowErrorInformation(m_dErrorCode);
	}
	
}

void CXWRobotDlg::OnBUTTONAEnable() 
{
	// TODO: Add your control notification handler code here
	BOOL oFault = FALSE;

	if(VCS_GetFaultState(m_KeyHandle,1,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,2,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,3,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,4,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,5,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,6,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,7,&oFault,&m_dErrorCode)&&
		VCS_GetFaultState(m_KeyHandle,8,&oFault,&m_dErrorCode))
	{
		
	}
	else
	{
		ShowErrorInformation(m_dErrorCode);
		return;
	}

	if(oFault)
	{
		if(VCS_ClearFault(m_KeyHandle,1,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,2,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,3,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,4,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,5,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,6,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,7,&m_dErrorCode)&&
			VCS_ClearFault(m_KeyHandle,8,&m_dErrorCode))
		{
		}
		else
		{
			ShowErrorInformation(m_dErrorCode);
			return;
		}
	}

	if(VCS_SetEnableState(m_KeyHandle,1,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,2,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,3,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,4,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,5,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,6,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,7,&m_dErrorCode)&&
		VCS_SetEnableState(m_KeyHandle,8,&m_dErrorCode)
		)
	{
		m_ADisable.EnableWindow(true);
		m_MoveAbsolute2.EnableWindow(true);
		m_MoveRelative2.EnableWindow(true);
		m_Walk.EnableWindow(true);
		m_Trot.EnableWindow(true);
		m_ModeSettings.EnableWindow(false);
		m_AEnable.EnableWindow(false);
	}
	else
	{
		ShowErrorInformation(m_dErrorCode);
	}
	
}

void CXWRobotDlg::OnBUTTONWalk() 
{
	// TODO: Add your control notification handler code here
# if ver==0
 adj=0;

 adj_flag=0;
 adjed_flag=0;//这些东西都记得归零啊！
 LFH_adj=0;
 LFK_adj=3*0;
 RFH_adj=0;
 RFK_adj=3*0;
 LHH_adj=0;
 LHK_adj=3*0;
 RHH_adj=0;
 RHK_adj=3*0;
# endif
	//Sleep(5000); //这里延迟5秒//这个先不关，看看看看新改的有没有用再注释掉
	loopt=0;
	stop_time=0;
	i=0;
	SetTimer(2,m_Period*1000/40,NULL);//这里还是暂时不改


	m_Pause.EnableWindow(true);
	m_Walk.EnableWindow(false);
	m_Trot.EnableWindow(false);
	m_ADisable.EnableWindow(false);
	m_MoveAbsolute2.EnableWindow(false);
	m_MoveRelative2.EnableWindow(false);
	m_Stop.EnableWindow(true);
	
}

void CXWRobotDlg::OnBUTTONTrot() 
{
	// TODO: Add your control notification handler code here
	SetTimer(3,m_Period*1000/20,NULL);
	m_Pause.EnableWindow(true);
	m_Walk.EnableWindow(false);
	m_Trot.EnableWindow(false);
	m_ADisable.EnableWindow(false);
	m_MoveAbsolute2.EnableWindow(false);
	m_MoveRelative2.EnableWindow(false);
	m_Stop.EnableWindow(true);
}

void CXWRobotDlg::OnBUTTONStop() 
{
	// TODO: Add your control notification handler code here
	# if ver==0
 adj=0;
 adj_flag=0;
 adjed_flag=0;//这些东西都记得归零啊！
 LFH_adj=0;
 LFK_adj=3*0;
 RFH_adj=0;
 RFK_adj=3*0;
 LHH_adj=0;
 LHK_adj=3*0;
 RHH_adj=0;
 RHK_adj=3*0;
# endif
	FirstIndex=1;
	i=0;
	stop_time=0;
	PauseIndex=1;
	KillTimer(2);
	KillTimer(3);
	if(!VCS_MoveToPosition(m_KeyHandle,1,LFHS,1,m_oImmediately,&m_dErrorCode))
	{
	    ShowErrorInformation(m_dErrorCode);
	}
					
	if(!VCS_MoveToPosition(m_KeyHandle,3,RFHS,1,m_oImmediately,&m_dErrorCode))
	{
        ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,6,RHKS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,8,LHKS,1,m_oImmediately,&m_dErrorCode))
	{
	    ShowErrorInformation(m_dErrorCode);
	}


	if(!VCS_MoveToPosition(m_KeyHandle,2,LFKS,1,m_oImmediately,&m_dErrorCode))
	{
	    ShowErrorInformation(m_dErrorCode);
	}					
	if(!VCS_MoveToPosition(m_KeyHandle,4,RFKS,1,m_oImmediately,&m_dErrorCode))
	{
        ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,5,RHHS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,7,LHHS,1,m_oImmediately,&m_dErrorCode))
	{
	    ShowErrorInformation(m_dErrorCode);
	}
	m_LF.SetPos(0);
	m_RF.SetPos(0);
	m_LH.SetPos(0);
	m_RH.SetPos(0);
	
	m_Continue.EnableWindow(false);
	m_Pause.EnableWindow(false);
    m_ADisable.EnableWindow(true);
	m_MoveAbsolute2.EnableWindow(true);
	m_MoveRelative2.EnableWindow(true);
	m_Stop.EnableWindow(false);
	m_Walk.EnableWindow(true);
	m_Trot.EnableWindow(true);

}

void CXWRobotDlg::OnBUTTONPause() 
{
	// TODO: Add your control notification handler code here
    PauseIndex=0;
	m_Continue.EnableWindow(true);
	m_Pause.EnableWindow(false);
	
}

void CXWRobotDlg::OnBUTTONContinue() 
{
	// TODO: Add your control notification handler code here
	PauseIndex=1;
    m_Continue.EnableWindow(false);
	m_Pause.EnableWindow(true);
}

void CXWRobotDlg::OnChangeEditH() 
{	
	// TODO: Add your control notification handler code here
	if(m_hWnd) UpdateData(true);
	OutputDebugString("\n here3");  
	Algorithm();
	
}


void CXWRobotDlg::OnEnChangeEditStartposition()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}



void CXWRobotDlg::OnEnChangeEdit1()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}

//以下是收发部分
void CXWRobotDlg::OnBnClickedButtonOpen()
{
	// TODO: 在此添加控件通知处理程序代码
	CString str, str1, n;

    GetDlgItemText(IDC_BUTTON_OPEN, str);   //str获得与对话框中的控件相关的标题或文本
    CWnd *h1;                               //窗口类的基类
    h1 = GetDlgItem(IDC_BUTTON_OPEN);       //指向控件的caption
	F1=0;F2=0;F3=0;F4=0;C1=0;C2=0;C3=0;G1=0;G2=0;G3=0;A1=0;A2=0;A3=0;
	T1=0;T2=0;T3=0;T4=0;T5=0;T6=0;T7=0;T8=0;//初始化数值们
    if(!m_mscom.get_PortOpen())
    {
        m_comb2.GetLBText(m_comb2.GetCurSel(), str1);   //取得所选的字符串，并存放在str1里面
        str1 = str1+','+'n'+','+'8'+','+'1';

        m_mscom.put_CommPort(m_comb1.GetCurSel()+1);    //选择串口
        m_mscom.put_InputMode(1);           //设置输入方式为二进制方式
        m_mscom.put_Settings(str1);         //comb2选择的波特率，无校验，8数据位，1个停止位
        m_mscom.put_InputLen(1024);         //设置当前接收区数据长度为1024
        m_mscom.put_RThreshold(1);          //缓冲区一个字符引发事件
        m_mscom.put_RTSEnable(1);           //设置RT允许

        m_mscom.put_PortOpen(true);         //打开串口
        //打开成功
		char strtime[15];
		 //尝试改名字	
		std::time_t result = std::time(nullptr);
		std::cout << std::asctime(std::localtime(&result))
				  << result << " seconds since the Epoch\n";
		FormatTime(result,strtime);
		    strcpy(str_nam,"E:\\robo\\data\\");
			strcat(str_nam,strtime);//"111");//
			strcat(str_nam,".txt");
		ofstream f1(str_nam,ios::app);//想改成时间命名
		if (! f1.is_open())		
		{  MessageBox(NULL,"Error opening file\n",MB_ICONERROR); exit (1); }   




        if(m_mscom.get_PortOpen())
        {
            str = _T("关闭STM");
            UpdateData(true);               //把控件中的值和变量进行交换 
            h1->SetWindowText(str);         //改变按钮名称为‘’关闭串口”,提供关闭操作
        }
    }
    //串口为打开状态
    else
    {
        m_mscom.put_PortOpen(false);        //关闭串口
        //关闭成功
		f1.close(); 
        if(str != _T("打开STM"))
        {
            str = _T("打开STM");
            UpdateData(true);
            h1->SetWindowText(str);         //更新数据后，提供打开操作
        }
    }
}


void CXWRobotDlg::OnBnClickedButtonSend()
{
	// TODO: 在此添加控件通知处理程序代码
    UpdateData(true);                       //更新控件数据
	CString sText;
	m_EditSend.GetWindowText(sText);
	m_mscom.put_Output(COleVariant(sText));
    //m_mscom.put_Output(COleVariant(m_EditSend.GetWindowTextA));//把发送编辑框的数据发送出去
}


void CXWRobotDlg::OnBnClickedButtonClean()
{
    e_EditReceive = _T("");                 //给接收编辑框发送空格符
    UpdateData(false);                      //根据数据状态反馈给控件
}


void CXWRobotDlg::OnBnClickedButtonClose()
{
    //若当前串口为打开
    if(m_mscom.get_PortOpen())
        m_mscom.put_PortOpen(false);        //置为关闭
}
BEGIN_EVENTSINK_MAP(CXWRobotDlg, CDialog)
	ON_EVENT(CXWRobotDlg, IDC_MSCOMM1, 1, CXWRobotDlg::OnCommMscomm1, VTS_NONE)
END_EVENTSINK_MAP()


void CXWRobotDlg::OnCommMscomm1()
{
	int num;
    //接收缓冲区有数据
    if(m_mscom.get_CommEvent() ==2 )
    {
		num=m_mscom.get_InBufferCount();//接收到的字符数目!
		if(num>=105){
			int temp = 0;
			int N = 0;
			char charArr[10] = {};
			char str[1024] = {0};           //缓冲区最大为1024
			long k;
			short cur1,cur2,cur3,cur4,cur5,cur6,cur7,cur8;
			//Sleep(100);//等.1s 笨方法，防止一次接收不完全(try
			VARIANT InputData = m_mscom.get_Input();    //读入缓冲区
			COleSafeArray fs;
			fs = InputData;                 //VARIANT型变À量转换为COleSafeArray型变量
			for(k = 0; k < fs.GetOneDimSize(); k++)
			{
				fs.GetElement(&k, str+k);   //转换为BYTE型数组
			}

			ofstream f1(str_nam,ios::app);//想改成时间命名
			if (! f1.is_open())  
			{  MessageBox(NULL,"Error opening file\n",MB_ICONERROR); exit (1); }  
			char strtime[15];
			std::time_t result = std::time(nullptr);
			FormatTime(result,strtime);
			//读电流数据
			VCS_GetCurrentIs(m_KeyHandle, 1, &cur1, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 2, &cur2, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 3, &cur3, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 4, &cur4, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 5, &cur5, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 6, &cur6, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 7, &cur7, &m_dErrorCode);
			VCS_GetCurrentIs(m_KeyHandle, 8, &cur8, &m_dErrorCode);
			
			f1<<strtime<<","<<cur1<<","<<cur2<<","<<cur3<<","<<cur4<<","<<cur5<<","<<cur6<<","<<cur7<<","<<cur8<<","<<str<<endl;			//写到文档f1
			
			//用于停止采集电流数据//f1<<strtime<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<str<<endl;			//写到文档f1
			
			e_EditReceive = str;           //接收到编辑框里面

		
	//下面把数据读进来
			char *tokenPtr=strtok(str,",");
			while(tokenPtr!=NULL)
			{
				switch (N)
				{
				case 0:
					F1=atof(tokenPtr);
					//f1<<"F1="<<F1+1<<endl;
					break;
				case 1:
					F2=atof(tokenPtr);
					//f1<<"F2="<<F2+1<<endl;
					break;
				case 2:
					F3=atof(tokenPtr);
					//f1<<"F3="<<F3+1<<endl;
					break;
				case 3:
					F4=atof(tokenPtr);
					//f1<<"F4="<<F4+1<<endl;//test
					break;
				case 4:
					C1=atof(tokenPtr);
					break;
				case 5:
					C2=atof(tokenPtr);
					break;
				case 6:
					C3=atof(tokenPtr);
					break;
				case 7:
					G1=atof(tokenPtr);
					break;
				case 8:
					G2=atof(tokenPtr);
					break;
				case 9:
					G3=atof(tokenPtr);
					break;
				case 10:
					A1=atof(tokenPtr);
					break;
				case 11:
					A2=atof(tokenPtr);
					break;
				case 12:
					A3=atof(tokenPtr);
					break;
			#ifdef encoder_set
				case 13:
					T1=atoi(tokenPtr);
					break;
				case 14:
					T2=atoi(tokenPtr);
					break;
				case 15:
					T3=atoi(tokenPtr);
					break;
				case 16:
					T4=atoi(tokenPtr);
					break;
				case 17:
					T5=atoi(tokenPtr);
					break;
				case 18:
					T6=atoi(tokenPtr);
					break;
				case 19:
					T7=atoi(tokenPtr);
					break;
				case 20:
					T8=atoi(tokenPtr);
					//f1<<"T8="<<T8+1<<endl;//test
					break;
			#endif
				}
				cout<<tokenPtr<<endl;//'\n';
				tokenPtr=strtok(NULL,",");
				N++;
			}
		#ifdef encoder_set
			if(N!=21)
		#else
			if(N!=13)
		#endif
			{
				//MessageBox(_T("传感器数据获取出错!"));
				return;
			}
			else
			{
				m_pitnum=m_s_pitnum+A2*100;
				m_rollnum=m_s_rollnum+A1*100;
				m_roll.SetPos((int)(A1));
				m_pit.SetPos((int)(A2));
			}
	//over
			UpdateData(false);              //将数据在屏幕中对应控件中显示出来。
		}
	}
}


void CXWRobotDlg::OnEnChangeEditRtposition()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CXWRobotDlg::OnBnClickedButtonAbsolute()
{
	// TODO: 在此添加控件通知处理程序代码

	if(VCS_GetPositionIs(m_KeyHandle,m_NodeID,&m_StartPosition,&m_dErrorCode))
	{
		if(!VCS_MoveToPosition(m_KeyHandle,m_NodeID,m_Target,1,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
		{
			ShowErrorInformation(m_dErrorCode);
		}
	}

	UpdateStatus();
}


void CXWRobotDlg::OnBnClickedButtonRemember()
{
	// TODO: 在此添加控件通知处理程序代码  changing
		BOOL oResult = m_oUpdateActive;
		oResult = VCS_GetPositionIs(m_KeyHandle,1,&LFHS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
		oResult = VCS_GetPositionIs(m_KeyHandle,2,&LFKS,&m_dErrorCode);
		//e_EditReceive = ltoa(LFKS);

		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
		oResult = VCS_GetPositionIs(m_KeyHandle,3,&RFHS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
		oResult = VCS_GetPositionIs(m_KeyHandle,4,&RFKS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
		oResult = VCS_GetPositionIs(m_KeyHandle,5,&RHHS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
		oResult = VCS_GetPositionIs(m_KeyHandle,6,&RHKS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}		
		oResult = VCS_GetPositionIs(m_KeyHandle,7,&LHHS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
		oResult = VCS_GetPositionIs(m_KeyHandle,8,&LHKS,&m_dErrorCode);
		if(!oResult)
		{
	        m_oUpdateActive = FALSE;
			ShowErrorInformation(m_dErrorCode);
		}
m_s_pitnum=-A2*100;
m_s_rollnum=-A1*100;
		//在测试窗口输出remember的初始角度（如果确定下来，还是建议来个上下电记忆法
		char s[60]=" ";
		sprintf(s,"%10d",LFHS);
		OutputDebugString("\n LFHS的记录数据=");
		OutputDebugString((LPCSTR)s);
		sprintf(s,"%10d",LFKS);
		OutputDebugString("\n LFKS的记录数据=");
		OutputDebugString((LPCSTR)s);

		sprintf(s,"%10d",RFHS);
		OutputDebugString("\n RFHS的记录数据=");
		OutputDebugString((LPCSTR)s);
		sprintf(s,"%10d",-1*RFKS);//-1是为了与m_RFK统一
		OutputDebugString("\n RFKS的记录数据=");
		OutputDebugString((LPCSTR)s);

		sprintf(s,"%10d",-1*RHHS);
		OutputDebugString("\n RHHS的记录数据=");
		OutputDebugString((LPCSTR)s);
		sprintf(s,"%10d",-1*RHKS);
		OutputDebugString("\n RHKS的记录数据=");
		OutputDebugString((LPCSTR)s);

		sprintf(s,"%10d",-1*LHHS);
		OutputDebugString("\n LHHS的记录数据=");
		OutputDebugString((LPCSTR)s);
		sprintf(s,"%10d",LHKS);
		OutputDebugString("\n LHKS的记录数据=");
		OutputDebugString((LPCSTR)s);
		
		sprintf(s,"%10d",m_s_pitnum);
		OutputDebugString("\n m_s_pitnum的记录数据=");
		OutputDebugString((LPCSTR)s);
		sprintf(s,"%10d",m_s_rollnum);
		OutputDebugString("\n m_s_rollnum的记录数据=");
		OutputDebugString((LPCSTR)s);
}




void CXWRobotDlg::OnClickedButtonMoverelative2()
{
	// TODO: 在此添加控件通知处理程序代码
		if(m_hWnd) UpdateData(true);
	if(!VCS_MoveToPosition(m_KeyHandle,1,m_LFH,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,2,m_LFK,0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,3,m_RFH,0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,4,m_RFK*(-1),0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,5,m_RHH*(-1),0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,6,m_RHK*(-1),0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,7,m_LHH*(-1),0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,8,m_LHK,0,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
}


void CXWRobotDlg::OnClickedButtonAbsolute2()
{
	// TODO: 在此添加控件通知处理程序代码
	if(m_hWnd) UpdateData(true);
	if(!VCS_MoveToPosition(m_KeyHandle,1,m_LFH+LFHS,1,m_oImmediately,&m_dErrorCode))//这里中间的1表示“绝对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,2,m_LFK+LFKS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,3,m_RFH+RFHS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,4,(m_RFK*(-1)+RFKS),1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,5,(m_RHH*(-1)+RHHS),1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,6,(m_RHK*(-1)+RHKS),1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,7,(m_LHH*(-1)+LHHS),1,m_oImmediately,&m_dErrorCode))
	{ 
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,8,m_LHK+LHKS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
}


void CXWRobotDlg::OnBnClickedButtonAbsolute3()
{
	// TODO: 在此添加控件通知处理程序代码
	if(m_hWnd) UpdateData(true);//这句话一定要加上，相当于读取输入的数据
		char s[40]=" ";
		sprintf(s,"%10d",m_LFH);
		OutputDebugString("\n LFH的记录数据=");
		OutputDebugString((LPCSTR)s);
}


void CXWRobotDlg::OnBnClickedButtonMoverelative3()
{
	// ALL+1000按钮 懒得改名
	if(m_hWnd) UpdateData(true);	//读取框里现在的数字
	m_LFH=m_LFH+1000;

	if(m_hWnd) UpdateData(false);	//显示出来

}




void CXWRobotDlg::OnBnClickedButtonMoverelative5()
{
	// LFH+
	if(!VCS_MoveToPosition(m_KeyHandle,1,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

void CXWRobotDlg::OnBnClickedButtonMoverelative13()
{
	// LFH-
	if(!VCS_MoveToPosition(m_KeyHandle,1,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative9()
{
	// LFK+
	if(!VCS_MoveToPosition(m_KeyHandle,2,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative17()
{
	// LFK-
	if(!VCS_MoveToPosition(m_KeyHandle,2,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

void CXWRobotDlg::OnBnClickedButtonMoverelative6()
{
	// RFH+
	if(!VCS_MoveToPosition(m_KeyHandle,3,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative14()
{
	// RFH-
	if(!VCS_MoveToPosition(m_KeyHandle,3,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}



void CXWRobotDlg::OnBnClickedButtonMoverelative10()
{
	// RFK+
	if(!VCS_MoveToPosition(m_KeyHandle,4,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative18()
{
	// RFK-
	if(!VCS_MoveToPosition(m_KeyHandle,4,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

void CXWRobotDlg::OnBnClickedButtonMoverelative7()
{
	// RHH+
	if(!VCS_MoveToPosition(m_KeyHandle,5,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}

void CXWRobotDlg::OnBnClickedButtonMoverelative15()
{
	// RHH-
	if(!VCS_MoveToPosition(m_KeyHandle,5,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative11()
{
	// RHK+
	if(!VCS_MoveToPosition(m_KeyHandle,6,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative19()
{
	// RHK-
	if(!VCS_MoveToPosition(m_KeyHandle,6,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative8()
{
	// LHH+
	if(!VCS_MoveToPosition(m_KeyHandle,7,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative16()
{
	// LHH-
	if(!VCS_MoveToPosition(m_KeyHandle,7,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative12()
{
	// LHK+
	if(!VCS_MoveToPosition(m_KeyHandle,8,867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative20()
{
	// LHK-
	if(!VCS_MoveToPosition(m_KeyHandle,8,-867*1,0,m_oImmediately,&m_dErrorCode))//这里中间的0表示“相对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
}


void CXWRobotDlg::OnBnClickedButtonMoverelative21()

{
	//reset
	i=0;
	stop_time=0;
# if ver==0
 adj=0;
 adj_flag=0;
 adjed_flag=0;//这些东西都记得归零啊！
 LFH_adj=0;
 LFK_adj=3*0;
 RFH_adj=0;
 RFK_adj=3*0;
 LHH_adj=0;
 LHK_adj=3*0;
 RHH_adj=0;
 RHK_adj=3*0;
# endif
	if(!VCS_MoveToPosition(m_KeyHandle,1,LFHS,1,m_oImmediately,&m_dErrorCode))//这里中间的1表示“绝对”运动
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,2,LFKS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,3,RFHS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,4,(RFKS),1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,5,(RHHS),1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,6,(RHKS),1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
	if(!VCS_MoveToPosition(m_KeyHandle,7,(LHHS),1,m_oImmediately,&m_dErrorCode))
	{ 
		ShowErrorInformation(m_dErrorCode);
	}
	if(!VCS_MoveToPosition(m_KeyHandle,8,LHKS,1,m_oImmediately,&m_dErrorCode))
	{
		ShowErrorInformation(m_dErrorCode);
	}	
}


void CXWRobotDlg::OnBnClickedCam()
{
	// TODO: 在此添加控件通知处理程序代码
	  	Mat image;
    image = imread("1.jpg");//argv[1], IMREAD_COLOR); // Read the file
       imshow("image", image);
	  while(1)
  {
      char c=cvWaitKey(33);
      if(c==27)break;		//等待按键ESC
  }
  //声明IplImage指针
  IplImage* pFrame = NULL;

 //获取摄像头
  CvCapture* pCapture = cvCreateCameraCapture(0);
 
  //创建窗口
  cvNamedWindow("video", 1);
 //

  //显示视屏
  int tim=99;
  while(tim--)
  {
      pFrame=cvQueryFrame( pCapture );
      if(!pFrame)break;
      cvShowImage("video",pFrame);
      char c=cvWaitKey(33);
      if(c==27)break;		//等待按键ESC
  }
  cvReleaseCapture(&pCapture);
  cvDestroyWindow("video");
}
