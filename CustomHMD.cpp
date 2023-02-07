#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>

#include <windows.h>
#include "openvr_driver.h"

#include "SerialRead.h"

using namespace vr;
using namespace std;

#define GRIP_BTN	0x0001
#define THUMB_BTN	0x0002
#define A_BTN		0x0004
#define B_BTN		0x0008
#define MENU_BTN	0x0010
#define SYS_BTN		0x0020

#define StepCtrlPos 0.0033;
#define StepPos     0.0033;
#define StepRot     0.4 * 0.5;

#define _X          0
#define _Y          1
#define _Z          2
#define _W          3

typedef struct _HMDData
{
    double  X, Y, Z;
    double  Yaw, Pitch, Roll;
} THMD, * PHMD;

typedef struct Matrix_
{
    double  M11, M12, M13, M14;
    double  M21, M22, M23, M24;
    double  M31, M32, M33, M34;
    double  M41, M42, M43, M44;
} Matrix;

typedef struct Vector_
{
    double  X;
    double  Y;
    double  Z;
    double  W;
} Vector;

typedef struct _Controller
{
    double	        X, Y, Z;
    double	        Yaw, Pitch, Roll;
    uint16_t        Buttons;
    float	        Trigger;
    float	        AxisX, AxisY, AxisZ;
} TController, * PController;

THMD        HMD;
TController FirstController, SecondController;

POINT PosMouse, PrevPosMouse, MouseOffset;

bool InitMouse = true;

Vector CameraLocation;
Vector RightVector, UpVector, ForwardVector;
Vector RightVector_, UpVector_, ForwardVector_;

class CSampleDeviceDriver : public ITrackedDeviceServerDriver, public IVRDisplayComponent
{
public:

    // keys for use with the settings API
    const char* k_pch_Sample_Section = "CustomHMD";
    const char* k_pch_Sample_SerialNumber_String = "serialNumber";
    const char* k_pch_Sample_ModelNumber_String = "modelNumber";
    const char* k_pch_SteamVR_IPD_Float = "ipd";
    const char* k_pch_Sample_WindowX_Int32 = "windowX";
    const char* k_pch_Sample_WindowY_Int32 = "windowY";
    const char* k_pch_Sample_WindowWidth_Int32 = "windowWidth";
    const char* k_pch_Sample_WindowHeight_Int32 = "windowHeight";
    const char* k_pch_Sample_RenderWidth_Int32 = "renderWidth";
    const char* k_pch_Sample_RenderHeight_Int32 = "renderHeight";
    const char* k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
    const char* k_pch_Sample_DisplayFrequency_Float = "displayFrequency";
    const char* k_pch_Sample_DistortionK1_Float = "DistortionK1";
    const char* k_pch_Sample_DistortionK2_Float = "DistortionK2";
    const char* k_pch_Sample_ZoomWidth_Float = "ZoomWidth";
    const char* k_pch_Sample_ZoomHeight_Float = "ZoomHeight";
    const char* k_pch_Sample_DistanceBetweenEyes_Int32 = "DistanceBetweenEyes";
    const char* k_pch_Sample_ScreenOffsetX_Int32 = "ScreenOffsetX";
    const char* k_pch_Sample_DebugMode_Bool = "DebugMode";

    TrackedDeviceIndex_t        m_unObjectId;
    PropertyContainerHandle_t   m_ulPropertyContainer;
    string                      m_sSerialNumber;
    string                      m_sModelNumber;
    float                       m_flIPD;
    int32_t                     m_nWindowX;
    int32_t                     m_nWindowY;
    int32_t                     m_nWindowWidth;
    int32_t                     m_nWindowHeight;
    int32_t                     m_nRenderWidth;
    int32_t                     m_nRenderHeight;
    float                       m_flSecondsFromVsyncToPhotons;
    float                       m_flDisplayFrequency;
    float                       m_fDistortionK1;
    float                       m_fDistortionK2;
    float                       m_fZoomWidth;
    float                       m_fZoomHeight;
    int32_t                     m_nDistanceBetweenEyes;
    int32_t                     m_nScreenOffsetX;
    bool                        m_bDebugMode;

    double  RAD(double Deg)
    {
        return (Deg * 0.017453292519943);
    }

    double  OffsetYPR(double Deg)
    {
        if (Deg < -180)
        {
            Deg += 360;
        }
        else if (Deg > 180)
        {
            Deg -= 360;
        }

        return (Deg);
    }

    CSampleDeviceDriver()
    {
        m_unObjectId = k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = k_ulInvalidPropertyContainer;
        m_sSerialNumber = "SerialNumber";
        m_sModelNumber = "ModelNumber";
        m_flIPD = VRSettings()->GetFloat(k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float);
        m_nWindowX = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowX_Int32);
        m_nWindowY = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowY_Int32);
        m_nWindowWidth = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowWidth_Int32);
        m_nWindowHeight = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowHeight_Int32);
        m_nRenderWidth = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_RenderWidth_Int32);
        m_nRenderHeight = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_RenderHeight_Int32);
        m_flSecondsFromVsyncToPhotons = VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float);
        m_flDisplayFrequency = VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float);
        m_fDistortionK1 = VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_DistortionK1_Float);
        m_fDistortionK2 = VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_DistortionK2_Float);
        m_fZoomWidth = VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_ZoomWidth_Float);
        m_fZoomHeight = VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_ZoomHeight_Float);
        m_nDistanceBetweenEyes = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_DistanceBetweenEyes_Int32);
        m_nScreenOffsetX = VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_ScreenOffsetX_Int32);
        m_bDebugMode = VRSettings()->GetBool(k_pch_Sample_Section, k_pch_Sample_DebugMode_Bool);
    }

    ~CSampleDeviceDriver()
    {

    }

    EVRInitError    Activate(TrackedDeviceIndex_t unObjectId)
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());
        VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD);
        VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
        VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency);
        VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

        // avoid "not fullscreen" warnings from vrmonitor
        VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

        //Debug mode activate Windowed Mode (borderless fullscreen), locked to 30 FPS, for testing
        VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DisplayDebugMode_Bool, m_bDebugMode);

        // Icons can be configured in code or automatically configured by an external file "drivername\resources\driver.vrresources".
        // Icon properties NOT configured in code (post Activate) are then auto-configured by the optional presence of a driver's "drivername\resources\driver.vrresources".
        // In this manner a driver can configure their icons in a flexible data driven fashion by using an external file.
        //
        // The structure of the driver.vrresources file allows a driver to specialize their icons based on their HW.
        // Keys matching the value in "Prop_ModelNumber_String" are considered first, since the driver may have model specific icons.
        // An absence of a matching "Prop_ModelNumber_String" then considers the ETrackedDeviceClass ("HMD", "Controller", "GenericTracker", "TrackingReference")
        // since the driver may have specialized icons based on those device class names.
        //
        // An absence of either then falls back to the "system.vrresources" where generic device class icons are then supplied.
        //
        // Please refer to "bin\drivers\sample\resources\driver.vrresources" which contains this sample configuration.
        //
        // "Alias" is a reserved key and specifies chaining to another json block.
        //
        // In this sample configuration file (overly complex FOR EXAMPLE PURPOSES ONLY)....
        //
        // "Model-v2.0" chains through the alias to "Model-v1.0" which chains through the alias to "Model-v Defaults".
        //
        // Keys NOT found in "Model-v2.0" would then chase through the "Alias" to be resolved in "Model-v1.0" and either resolve their or continue through the alias.
        // Thus "Prop_NamedIconPathDeviceAlertLow_String" in each model's block represent a specialization specific for that "model".
        // Keys in "Model-v Defaults" are an example of mapping to the same states, and here all map to "Prop_NamedIconPathDeviceOff_String".

        // Setup properties directly in code.
        // Path values are of the form {drivername}\icons\some_icon_filename.png
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceOff_String, "{CustomHMD}/icons/headset_sample_status_off.png");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceSearching_String, "{CustomHMD}/icons/headset_sample_status_searching.gif");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String, "{CustomHMD}/icons/headset_sample_status_searching_alert.gif");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceReady_String, "{CustomHMD}/icons/headset_sample_status_ready.png");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceReadyAlert_String, "{CustomHMD}/icons/headset_sample_status_ready_alert.png");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceNotReady_String, "{CustomHMD}/icons/headset_sample_status_error.png");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceStandby_String, "{CustomHMD}/icons/headset_sample_status_standby.png");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceAlertLow_String, "{CustomHMD}/icons/headset_sample_status_ready_low.png");

        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceOff_String,           "{CustomHMD}/icons/controller_status_off.png");
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceSearching_String,     "{CustomHMD}/icons/controller_status_searching.gif");
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String,"{CustomHMD}/icons/controller_status_searching_alert.gif");
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceReady_String,         "{CustomHMD}/icons/controller_status_ready.png");
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceReadyAlert_String,    "{CustomHMD}/icons/controller_status_ready_alert.png");
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceNotReady_String,      "{CustomHMD}/icons/controller_status_error.png");
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceStandby_String,       "{CustomHMD}/icons/controller_status_standby.png",);
        // VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceAlertLow_String,      "{CustomHMD}/icons/controller_status_ready_low.png");

        return VRInitError_None;
    }

    void    Deactivate()
    {
        m_unObjectId = k_unTrackedDeviceIndexInvalid;
    }

    void    EnterStandby()
    {

    }

    void* GetComponent(const char* pchComponentNameAndVersion)
    {
        if (!_stricmp(pchComponentNameAndVersion, IVRDisplayComponent_Version))
        {
            return (IVRDisplayComponent*)this;
        }

        // override this to add a component to a driver
        return NULL;
    }

    void    PowerOff()
    {

    }

    void    DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }

    void    GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
    {
        *pnX = m_nWindowX;
        *pnY = m_nWindowY;
        *pnWidth = m_nWindowWidth;
        *pnHeight = m_nWindowHeight;
    }

    bool    IsDisplayOnDesktop()
    {
        return true;
    }

    bool    IsDisplayRealDisplay()
    {
        return false;
    }

    void    GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
    {
        *pnWidth = m_nRenderWidth;
        *pnHeight = m_nRenderHeight;
    }

    void    GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
    {
        *pnY = m_nScreenOffsetX;
        *pnWidth = m_nWindowWidth / 2;
        *pnHeight = m_nWindowHeight;

        if (eEye == Eye_Left)
        {
            *pnX = m_nDistanceBetweenEyes;
        }
        else
        {
            *pnX = (m_nWindowWidth / 2) - m_nDistanceBetweenEyes;
        }
    }

    void    GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
    {
        *pfLeft = -1.0;
        *pfRight = 1.0;
        *pfTop = -1.0;
        *pfBottom = 1.0;
    }

    DistortionCoordinates_t     ComputeDistortion(EVREye eEye, float fU, float fV)
    {
        DistortionCoordinates_t coordinates;

        // Distortion for lens from https://github.com/HelenXR/openvr_survivor/blob/master/src/head_mount_display_device.cc

        float  hX;
        float  hY;
        double rr;
        double r2;
        double theta;

        rr = sqrt((fU - 0.5f) * (fU - 0.5f) + (fV - 0.5f) * (fV - 0.5f));
        r2 = rr * (1 + m_fDistortionK1 * (rr * rr) + m_fDistortionK2 * (rr * rr * rr * rr));
        theta = atan2(fU - 0.5f, fV - 0.5f);
        hX = sin(theta) * r2 * m_fZoomWidth;
        hY = cos(theta) * r2 * m_fZoomHeight;

        coordinates.rfBlue[0] = hX + 0.5f;
        coordinates.rfBlue[1] = hY + 0.5f;
        coordinates.rfGreen[0] = hX + 0.5f;
        coordinates.rfGreen[1] = hY + 0.5f;
        coordinates.rfRed[0] = hX + 0.5f;
        coordinates.rfRed[1] = hY + 0.5f;

        return coordinates;
    }

    // Headset Control
    DriverPose_t        GetPose()
    {
        DriverPose_t    pose = { 0 };

        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        pose.qWorldFromDriverRotation.w = 1;
        pose.qWorldFromDriverRotation.x = 0;
        pose.qWorldFromDriverRotation.y = 0;
        pose.qWorldFromDriverRotation.z = 0;

        pose.qDriverFromHeadRotation.w = 1;
        pose.qDriverFromHeadRotation.x = 0;
        pose.qDriverFromHeadRotation.y = 0;
        pose.qDriverFromHeadRotation.z = 0;

        MouseManagement();

        if ((GetKeyState(VK_LBUTTON) & 0x100) == 0 && (GetKeyState(VK_RBUTTON) & 0x100) == 0)
        {
            // Rotate Right/Left
            HMD.Pitch -= MouseOffset.x * StepRot;

            // Rotate Up/Down
            HMD.Roll -= MouseOffset.y * StepRot;

            // Roll Left/Right
            //HMD.Yaw   += StepRot;
        }

        HMD.Yaw = OffsetYPR(HMD.Yaw);
        HMD.Pitch = OffsetYPR(HMD.Pitch);
        HMD.Roll = OffsetYPR(HMD.Roll);

        //Convert yaw, pitch, roll to quaternion
        Vector Cos = { cos(RAD(HMD.Pitch)) , cos(RAD(HMD.Yaw)) , cos(RAD(HMD.Roll)) , 0 };
        Vector Sin = { sin(RAD(HMD.Pitch)) , sin(RAD(HMD.Yaw)) , sin(RAD(HMD.Roll)) , 0 };

        //Set head tracking rotation
        pose.qRotation.w = (Cos.Y * Cos.Z * Cos.X) + (Sin.Y * Sin.Z * Sin.X);
        pose.qRotation.x = (Cos.Y * Sin.Z * Cos.X) - (Sin.Y * Cos.Z * Sin.X);
        pose.qRotation.y = (Cos.Y * Cos.Z * Sin.X) + (Sin.Y * Sin.Z * Cos.X);
        pose.qRotation.z = (Sin.Y * Cos.Z * Cos.X) - (Cos.Y * Sin.Z * Sin.X);

        // Move Forward/Backward
        if ((GetAsyncKeyState('Y') & 0x8000) != 0)  CameraLocation.Z -= StepPos;
        if ((GetAsyncKeyState(' ') & 0x8000) != 0)  CameraLocation.Z += StepPos;

        // Move Right/Left
        if ((GetAsyncKeyState('A') & 0x8000) != 0)  CameraLocation.X -= StepPos;
        if ((GetAsyncKeyState('D') & 0x8000) != 0)  CameraLocation.X += StepPos;

        // Move Up/Down
        if ((GetAsyncKeyState('E') & 0x8000) != 0)  CameraLocation.Y -= StepPos;
        if ((GetAsyncKeyState('Z') & 0x8000) != 0)  CameraLocation.Y += StepPos;

        //std::string myString = std::to_string(CameraVector.Z);
        //LPWSTR ws = new wchar_t[myString.size() + 1];
        //copy(myString.begin(), myString.end(), ws);
        //ws[myString.size()] = 0;
        //{
        //    MessageBox(NULL, ws, L"Windows Tutorial", MB_OK);
        //}

        pose.vecPosition[_X] = CameraLocation.X;
        pose.vecPosition[_Y] = CameraLocation.Y;
        pose.vecPosition[_Z] = CameraLocation.Z;

        return pose;
    }

    void    MouseManagement()
    {
        POINT PosMouse;
        RECT  rect, rect_;

        GetWindowRect(GetDesktopWindow(), &rect);
        {                                               // LeftX/TopY_________
            rect_.left = (rect.right / 2) - 400;     //      |             |
            rect_.top = (rect.bottom / 2) - 400;     //      |             |
            rect_.right = (rect.right / 2) + 400;     //      |             |
            rect_.bottom = (rect.bottom / 2) + 400;     //      |_______RightX/BottomY
        }
        ClipCursor(&rect_);

        if (InitMouse)
        {
            SetCursorPos(rect.right / 2, rect.bottom / 2);

            GetCursorPos(&PrevPosMouse);
            GetCursorPos(&PosMouse);

            InitMouse = false;
        }

        rect_.right--;
        rect_.bottom--;

        GetCursorPos(&PosMouse);
        {
            if (PosMouse.x >= rect_.right || PosMouse.y >= rect_.bottom ||
                PosMouse.x <= rect_.left || PosMouse.y <= rect_.top)
            {
                if (PosMouse.x >= rect_.right)  SetCursorPos(PosMouse.x - 800, PosMouse.y);
                else if (PosMouse.y >= rect_.bottom) SetCursorPos(PosMouse.x, PosMouse.y - 800);
                else if (PosMouse.x <= rect_.left)   SetCursorPos(PosMouse.x + 800, PosMouse.y);
                else if (PosMouse.y <= rect_.top)    SetCursorPos(PosMouse.x, PosMouse.y + 800);

                GetCursorPos(&PrevPosMouse);
                GetCursorPos(&PosMouse);
            }

            MouseOffset.x = PosMouse.x - PrevPosMouse.x;
            MouseOffset.y = PosMouse.y - PrevPosMouse.y;
            PrevPosMouse.x = PosMouse.x;
            PrevPosMouse.y = PosMouse.y;
        }
    }

    void    RunFrame()
    {
        // In a real driver, this should happen from some pose tracking thread.
        // The RunFrame interval is unspecified and can be very irregular if some other
        // driver blocks it for some periodic task.

        VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
    }

    string GetSerialNumber() const { return m_sSerialNumber; }
};

class CSampleControllerDriver : public ITrackedDeviceServerDriver
{
public:

    int32_t                     ControllerIndex;
    TrackedDeviceIndex_t        m_unObjectId;
    PropertyContainerHandle_t   m_ulPropertyContainer;
    VRInputComponentHandle_t    m_compHaptic;
    VRInputComponentHandle_t    ulSkeletalComponentHandle;
    VRInputComponentHandle_t    HButtons[4], HAnalog[3];

    double  RAD(double Deg)
    {
        return (Deg * 0.017453292519943);
    }

    double  OffsetYPR(double Deg)
    {
        if (Deg < -180)
        {
            Deg += 360;
        }
        else if (Deg > 180)
        {
            Deg -= 360;
        }

        return (Deg);
    }

    CSampleControllerDriver()
    {
        m_unObjectId = k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = k_ulInvalidPropertyContainer;
    }

    void    SetControllerIndex(int32_t CtrlIndex)
    {
        ControllerIndex = CtrlIndex;
    }

    ~CSampleControllerDriver()
    {

    }

    EVRInitError    Activate(TrackedDeviceIndex_t unObjectId)
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ControllerType_String, "vive_controller");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "vr_controller_vive_1_5");

        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");
        VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);

        switch (ControllerIndex)
        {
        case 1:
            VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL1Serial");
            break;

        case 2:
            VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL2Serial");
            break;
        }

        uint64_t supportedButtons = 0xFFFFFFFFFFFFFFFFULL;
        VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_SupportedButtons_Uint64, supportedButtons);
        VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        //VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        // avoid "not fullscreen" warnings from vrmonitor
        //VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false );

        // our sample device isn't actually tracked, so set this property to avoid having the icon blink in the status window
        //VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_NeverTracked_Bool, false );

        // even though we won't ever track we want to pretend to be the right hand so binding will work as expected

        switch (ControllerIndex)
        {
        case 1:
            VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand);
            break;

        case 2:
            VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
            break;
        }

        // this file tells the UI what to show the user for binding this controller as well as what default bindings should
        // be for legacy or other apps
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{CustomHMD}/input/CustomHMD_profile.json");

        // Create all the input components
            // Buttons handles
        VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &HButtons[0]);
        VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &HButtons[1]);
        VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[2]);
        VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtons[3]);

        // Analog handles
        VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalog[0], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalog[1], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalog[2], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);

        // create our haptic component
        VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

        return VRInitError_None;
    }

    void    Deactivate()
    {
        m_unObjectId = k_unTrackedDeviceIndexInvalid;
    }

    void    EnterStandby()
    {

    }

    void* GetComponent(const char* pchComponentNameAndVersion)
    {
        // override this to add a component to a driver
        return NULL;
    }

    void    PowerOff()
    {

    }

    void    DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }

    // Controller Control
    DriverPose_t        GetPose()
    {
        DriverPose_t    pose = { 0 };

        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        pose.qWorldFromDriverRotation.w = 1;
        pose.qWorldFromDriverRotation.x = 0;
        pose.qWorldFromDriverRotation.y = 0;
        pose.qWorldFromDriverRotation.z = 0;

        pose.qDriverFromHeadRotation.w = 1;
        pose.qDriverFromHeadRotation.x = 0;
        pose.qDriverFromHeadRotation.y = 0;
        pose.qDriverFromHeadRotation.z = 0;

        if (ControllerIndex == 1)
        {
            // FirstController
                // Move Forward/Backward
            if ((GetAsyncKeyState('Y') & 0x8000) != 0)  FirstController.Z -= StepCtrlPos;
            if ((GetAsyncKeyState(' ') & 0x8000) != 0)  FirstController.Z += StepCtrlPos;

            // Move Right/Left
            if ((GetAsyncKeyState('A') & 0x8000) != 0)  FirstController.X -= StepCtrlPos;
            if ((GetAsyncKeyState('D') & 0x8000) != 0)  FirstController.X += StepCtrlPos;

            // Move Up/Down
            if ((GetAsyncKeyState('Z') & 0x8000) != 0)  FirstController.Y += StepCtrlPos;
            if ((GetAsyncKeyState('E') & 0x8000) != 0)  FirstController.Y -= StepCtrlPos;

            if ((GetKeyState(VK_LBUTTON) & 0x100) != 0)
            {
                // Rotate Right/Left
                FirstController.Pitch -= MouseOffset.x * StepRot;

                // Rotate Up/Down
                FirstController.Roll -= MouseOffset.y * StepRot;

                // Roll Left/Right
                //FirstController.Yaw   += StepRot;
            }

            FirstController.Yaw = OffsetYPR(FirstController.Yaw);
            FirstController.Pitch = OffsetYPR(FirstController.Pitch);
            FirstController.Roll = OffsetYPR(FirstController.Roll);

            //Convert yaw, pitch, roll to quaternion
            Vector Cos = { cos(RAD(FirstController.Pitch)) , cos(RAD(FirstController.Yaw)) , cos(RAD(FirstController.Roll)) , 0 };
            Vector Sin = { sin(RAD(FirstController.Pitch)) , sin(RAD(FirstController.Yaw)) , sin(RAD(FirstController.Roll)) , 0 };

            //Convert yaw, pitch, roll to quaternion
            //Set controller tracking rotation
            pose.qRotation.w = (Cos.Y * Cos.Z * Cos.X) + (Sin.Y * Sin.Z * Sin.X);
            pose.qRotation.x = (Cos.Y * Sin.Z * Cos.X) - (Sin.Y * Cos.Z * Sin.X);
            pose.qRotation.y = (Cos.Y * Cos.Z * Sin.X) + (Sin.Y * Sin.Z * Cos.X);
            pose.qRotation.z = (Sin.Y * Cos.Z * Cos.X) - (Cos.Y * Sin.Z * Sin.X);

            pose.vecPosition[0] = FirstController.X - 0.2;
            pose.vecPosition[1] = FirstController.Y;
            pose.vecPosition[2] = FirstController.Z - 0.5;
        }
        else
        {
            // SecondController
            if (!DeviceConnected) SerialBegin(L"COM4");

            SecondController.Yaw = SerialRead("yaw");
            SecondController.Pitch = SerialRead("pitch");
            SecondController.Roll = SerialRead("roll");
            SecondController.X = SerialRead("x");
            SecondController.Y = SerialRead("y");
            SecondController.Z = SerialRead("z");

            SecondController.Yaw = OffsetYPR(SecondController.Yaw);
            SecondController.Pitch = OffsetYPR(SecondController.Pitch);
            SecondController.Roll = OffsetYPR(SecondController.Roll);

            //Convert yaw, pitch, roll to quaternion
            Vector Cos = { cos(RAD(SecondController.Pitch)) , cos(RAD(SecondController.Yaw)) , cos(RAD(SecondController.Roll)) , 0 };
            Vector Sin = { sin(RAD(SecondController.Pitch)) , sin(RAD(SecondController.Yaw)) , sin(RAD(SecondController.Roll)) , 0 };

            //Convert yaw, pitch, roll to quaternion
            //Set controller tracking rotation
            pose.qRotation.w = (Cos.Y * Cos.Z * Cos.X) + (Sin.Y * Sin.Z * Sin.X);
            pose.qRotation.x = (Cos.Y * Sin.Z * Cos.X) - (Sin.Y * Cos.Z * Sin.X);
            pose.qRotation.y = (Cos.Y * Cos.Z * Sin.X) + (Sin.Y * Sin.Z * Cos.X);
            pose.qRotation.z = (Sin.Y * Cos.Z * Cos.X) - (Cos.Y * Sin.Z * Sin.X);

            pose.vecPosition[0] = SecondController.X - 0.2;
            pose.vecPosition[1] = SecondController.Y;
            pose.vecPosition[2] = SecondController.Z - 0.5;

            pose.vecPosition[0] = SecondController.X + 0.2;
            pose.vecPosition[1] = SecondController.Y;
            pose.vecPosition[2] = SecondController.Z - 0.5;
        }

        return pose;
    }

    void    RunFrame()
    {
        // Your driver would read whatever hardware state is associated with its input components and pass that
        // in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
        // state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.

        if (ControllerIndex == 1)
        {
            VRDriverInput()->UpdateBooleanComponent(HButtons[0], (0x8000 & GetAsyncKeyState(0x39)) != 0, 0); //Application Menu
            VRDriverInput()->UpdateBooleanComponent(HButtons[1], (0x8000 & GetAsyncKeyState(0x30)) != 0, 0); //Grip
            VRDriverInput()->UpdateBooleanComponent(HButtons[2], (0x8000 & GetAsyncKeyState(0xDB)) != 0, 0); //System
            VRDriverInput()->UpdateBooleanComponent(HButtons[3], (0x8000 & GetAsyncKeyState(0xDD)) != 0, 0); //Trackpad

            VRDriverInput()->UpdateScalarComponent(HAnalog[0], 0.0, 0); //Trackpad x
            VRDriverInput()->UpdateScalarComponent(HAnalog[1], 0.0, 0); //Trackpad y

            if ((GetAsyncKeyState('W') & 0x8000) != 0)
                VRDriverInput()->UpdateScalarComponent(HAnalog[0], 1.0, 0);

            if ((GetAsyncKeyState('X') & 0x8000) != 0)
                VRDriverInput()->UpdateScalarComponent(HAnalog[1], 1.0, 0);

            if ((GetAsyncKeyState('C') & 0x8000) != 0) //Trigger
            {
                VRDriverInput()->UpdateScalarComponent(HAnalog[2], 1.0, 0);
            }
            else
            {
                VRDriverInput()->UpdateScalarComponent(HAnalog[2], 0.0, 0);
            }
        }
        else
        {
            VRDriverInput()->UpdateBooleanComponent(HButtons[0], (0x8000 & GetAsyncKeyState(VK_F1)) != 0, 0); //Application Menu
            VRDriverInput()->UpdateBooleanComponent(HButtons[1], (0x8000 & GetAsyncKeyState(VK_F2)) != 0, 0); //Grip
            VRDriverInput()->UpdateBooleanComponent(HButtons[2], (0x8000 & GetAsyncKeyState(VK_F3)) != 0, 0); //System
            VRDriverInput()->UpdateBooleanComponent(HButtons[3], (0x8000 & GetAsyncKeyState(VK_F4)) != 0, 0); //Trackpad

            VRDriverInput()->UpdateScalarComponent(HAnalog[0], 0.0, 0); //Trackpad x
            VRDriverInput()->UpdateScalarComponent(HAnalog[1], 0.0, 0); //Trackpad y

            if ((GetAsyncKeyState('V') & 0x8000) != 0) //Trigger
            {
                VRDriverInput()->UpdateScalarComponent(HAnalog[2], 1.0, 0);
            }
            else
            {
                VRDriverInput()->UpdateScalarComponent(HAnalog[2], 0.0, 0);
            }
        }

        if (m_unObjectId != k_unTrackedDeviceIndexInvalid)
        {
            VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
        }
    }

    void    ProcessEvent(const VREvent_t& vrEvent)
    {
        switch (vrEvent.eventType)
        {
        case VREvent_Input_HapticVibration:
        {
            if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
            {
                // This is where you would send a signal to your hardware to trigger actual haptic feedback
            }
        }
        break;
        }
    }

    string GetSerialNumber() const
    {
        switch (ControllerIndex)
        {
        case 1:
            return "CTRL1Serial";
            break;

        case 2:
            return "CTRL2Serial";
            break;
        }
    }
};

class CServerDriver_Sample : public IServerTrackedDeviceProvider
{
public:

    CSampleDeviceDriver* m_pNullHmdLatest;
    CSampleControllerDriver* m_pController;
    CSampleControllerDriver* m_pController2;

    const char* const* GetInterfaceVersions() { return k_InterfaceVersions; }
    bool ShouldBlockStandbyMode() { return false; }
    void EnterStandby() {}
    void LeaveStandby() {}

    EVRInitError Init(IVRDriverContext* pDriverContext)
    {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

        m_pNullHmdLatest = new CSampleDeviceDriver();
        VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), TrackedDeviceClass_HMD, m_pNullHmdLatest);

        m_pController = new CSampleControllerDriver();
        m_pController->SetControllerIndex(1);
        VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), TrackedDeviceClass_Controller, m_pController);

        m_pController2 = new CSampleControllerDriver();
        m_pController2->SetControllerIndex(2);
        VRServerDriverHost()->TrackedDeviceAdded(m_pController2->GetSerialNumber().c_str(), TrackedDeviceClass_Controller, m_pController2);

        return VRInitError_None;
    }

    void Cleanup()
    {
        delete m_pNullHmdLatest;
        m_pNullHmdLatest = NULL;

        delete m_pController;
        m_pController = NULL;

        delete m_pController2;
        m_pController2 = NULL;
    }

    void RunFrame()
    {
        m_pNullHmdLatest->RunFrame();
        m_pController->RunFrame();
        m_pController2->RunFrame();

        VREvent_t vrEvent;
        while (VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
        {
            m_pController->ProcessEvent(vrEvent);
            m_pController2->ProcessEvent(vrEvent);
        }
    }
};

// This is the main for the driver, this is how openvr finds out about what kinds of devices your driver provides.
// Conceptually, Openvr considers your driver to be a device provider and you provide a IServerTrackedDeviceProvider
// that openvr will give an IVRDriverContext to.
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#define HMD_DLL_IMPORT extern "C" __declspec(dllimport)

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int32_t* pReturnCode)
{
    static CServerDriver_Sample g_serverDriverNull;

    return &g_serverDriverNull;
}