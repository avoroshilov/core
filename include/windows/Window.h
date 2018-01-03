#pragma once

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <Windows.h>

namespace windows
{

typedef void (* pfnResizeCallback)(void * pUserData, int width, int height);
typedef void (* pfnChangeFocusCallback)(void * pUserData, bool isInFocus);
typedef void (* pfnChangeActiveCallback)(void * pUserData, bool isActive);
typedef void (* pfnCloseCallback)(void * pUserData);

enum class KeyState
{
	eReleased,
	ePressed,
	eHeldDown,
	
	eNUM_ENTRIES
};
enum class KeyCode
{
	eSPECIAL_KEYCODE_REGION = 256,

	eEscape,
	eLShift,
	eRShift,
	eLAlt,
	eRAlt,
	eLCtrl,
	eRCtrl,

	eEnter,
	eSpace,
	eTab,

	eInsert,
	eDelete,
	eHome,
	eEnd,
	ePgUp,
	ePgDown,

	eF1,
	eF2,
	eF3,
	eF4,
	eF5,
	eF6,
	eF7,
	eF8,
	eF9,
	eF10,
	eF11,
	eF12,

	eNUM_ENTRIES
};
typedef void (* pfnKeyStateCallback)(void * pUserData, KeyCode keyCode, KeyState keyDownState);
struct MouseEvent
{
	enum class Type
	{
		eMove,
		eLBDown,
		eLBUp,
		eMBDown,
		eMBUp,
		eRBDown,
		eRBUp,
		eWheel,

		eNUM_ENTRIES
	};

	Type type;
	int dX;
	int dY;
};
typedef void (* pfnMouseEventCallback)(void * pUserData, MouseEvent mouseEvent);

void setUserDataPointer(void * pUserData);
void setResizeCallback(pfnResizeCallback callback);
void setChangeFocusCallback(pfnChangeFocusCallback callback);
void setChangeActiveCallback(pfnChangeActiveCallback callback);
void setCloseCallback(pfnCloseCallback callback);
void setKeyStateCallback(pfnKeyStateCallback callback);
void setMouseEventCallback(pfnMouseEventCallback callback);

void getMouseCoordinates(int * mx, int * my);
void setMouseCoordinates(int mx, int my);
void showMouse();
void hideMouse();

class Window
{
	HINSTANCE	m_hInstance;
	HWND		m_hWnd;

	enum class RawInputDeviceType
	{
		eKeyboard,
		eMouse,
		
		eNUM_ENTRIES
	};
	int m_riKeyboardIndex = -1;
	int m_riMouseIndex = -1;
	std::vector<RAWINPUTDEVICE> m_riDevices;

	HDC m_hDC = nullptr;
	bool m_ownsDC = false;

	static const int m_classnameBufSize = 256;
	wchar_t m_classname[m_classnameBufSize];

	static const int m_titleBufSize = 256;
	wchar_t m_title[m_titleBufSize];
	
	bool m_isResizeable = true;

	uint32_t m_width, m_height;
	uint32_t m_colorBits;
	
	static uint32_t windowsCount;

	static const size_t keyStatesNum = 256;
	char m_keyStates[keyStatesNum];

public:

	void setKeyState(int vKeyCode, char state)
	{
		if (vKeyCode < (int)keyStatesNum)
		{
			m_keyStates[vKeyCode] = state;
		}
	}
	char getKeyState(int vKeyCode)
	{
		if (vKeyCode < (int)keyStatesNum)
		{
			return m_keyStates[vKeyCode];
		}
		return 0;
	}

	Window()
	{
		m_windowKind = Kind::eWindowed;
		m_colorBits = 32;
		swprintf_s(m_classname, m_classnameBufSize, L"WindowClass %d", ++windowsCount);
		memset(m_keyStates, 0, keyStatesNum*sizeof(char));
	}

	HINSTANCE	getHInstance() const	{ return m_hInstance; }
	HWND		getHWnd() const			{ return m_hWnd; }

	uint32_t	getWidth() const		{ return m_width; }
	uint32_t	getHeight() const		{ return m_height; }

	void setResizeable(bool isResizeable) { m_isResizeable = isResizeable; }
	bool getResizeable() const { return m_isResizeable; }

	enum class Kind
	{
		eWindowed,
		eFullscreen,
		eFullscreenBorderless,

		eNUM_ENTRIES
	};
	Kind m_windowKind;

	void setParameters(uint32_t width, uint32_t height, Kind windowKind = Kind::eWindowed);
	void setTitle(const wchar_t * title, ...);

	HDC getDC() const { return m_hDC; }

	bool init(bool ownsDC = false);
	void deinit();
};

}
