#include <stdio.h>

#include "windows/window.h"

typedef uint32_t uint;

namespace windows
{
	LRESULT CALLBACK WindowProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

	void * g_pUserData = nullptr;
	void setUserDataPointer(void * pUserData)
	{
		g_pUserData = pUserData;
	}

	pfnResizeCallback g_pfnResizeCallback = nullptr;
	void setResizeCallback(pfnResizeCallback callback)
	{
		g_pfnResizeCallback = callback;
	}

	pfnChangeFocusCallback g_pfnChangeFocusCallback = nullptr;
	void setChangeFocusCallback(pfnChangeFocusCallback callback)
	{
		g_pfnChangeFocusCallback = callback;
	}

	pfnCloseCallback g_pfnCloseCallback = nullptr;
	void setCloseCallback(pfnCloseCallback callback)
	{
		g_pfnCloseCallback = callback;
	}

	pfnKeyStateCallback g_pfnKeyStateCallback = nullptr;
	void setKeyStateCallback(pfnKeyStateCallback callback)
	{
		g_pfnKeyStateCallback = callback;
	}


	uint Window::windowsCount = 0;

	void Window::setParameters(uint32_t width, uint32_t height, bool fullscreen)
	{
		m_width = width;
		m_height = height;
		m_fullscreen = fullscreen;
	}

	void Window::setTitle(const wchar_t * title, ...)
	{
		const int titleTextBufSize = 256;
		wchar_t titleText[titleTextBufSize];

		va_list ap;

		if (title == NULL)
			return;

		va_start(ap, title);
			vswprintf_s(titleText, titleTextBufSize, title, ap);
		va_end(ap);

		SetWindowText(m_hWnd, titleText);
	}

	bool Window::init(bool ownsDC)
	{
		WNDCLASSEX				wcx;
		RECT					rect;
		DWORD					style, exStyle;
		int						x, y;

		m_ownsDC = ownsDC;

		m_hInstance = static_cast<HINSTANCE>(GetModuleHandle(NULL));

		memset(&wcx, 0, sizeof(wcx));

		wcx.cbSize			= sizeof(wcx);
		wcx.style			= CS_HREDRAW | CS_VREDRAW;
		wcx.lpfnWndProc		= reinterpret_cast<WNDPROC>(WindowProc);
		wcx.cbClsExtra		= 0;
		wcx.cbWndExtra		= 0;
		wcx.hInstance		= m_hInstance;
		wcx.hIcon			= LoadIcon(NULL, IDI_APPLICATION);
		wcx.hCursor			= LoadCursor(NULL, IDC_ARROW);
		wcx.hbrBackground	= (HBRUSH)GetStockObject(BLACK_BRUSH);
		wcx.lpszMenuName	= NULL;
		wcx.lpszClassName	= m_classname;
		wcx.hIconSm			= LoadIcon(NULL, IDI_WINLOGO);

		if (m_ownsDC)
		{
			wcx.style |= CS_OWNDC;
		}

		if (!RegisterClassEx(&wcx))
		{
			return false;
		}

		int screenWidth = GetSystemMetrics(SM_CXSCREEN);
		int screenHeight = GetSystemMetrics(SM_CYSCREEN);
		x = (screenWidth - m_width)  / 2;
		y = (screenHeight - m_height) / 2;

		if (m_fullscreen)
		{
			DEVMODE dmScreenSettings;
			memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));
			dmScreenSettings.dmSize = sizeof(dmScreenSettings);
			dmScreenSettings.dmPelsWidth = screenWidth;
			dmScreenSettings.dmPelsHeight = screenHeight;
			dmScreenSettings.dmBitsPerPel = 32;
			dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

			if ((m_width != (uint32_t)screenWidth) && (m_height != (uint32_t)screenHeight))
			{
				if (ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN) != DISP_CHANGE_SUCCESSFUL)
				{
					m_fullscreen = false;
					// TODO: warning
					printf("Fullscreen is not supprted!\n");
				}
			}
		}

		if (m_fullscreen)
		{
			// See style definitions below
			style = WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
			exStyle = WS_EX_APPWINDOW;

			rect.left = 0;
			rect.right = screenWidth;
			rect.top = 0;
			rect.bottom = screenHeight;
		}
		else
		{
			// WS_OVERLAPPEDWINDOW = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX
			// WS_CAPTION - window includes title bar (includes the WS_BORDER style)
			// WS_OVERLAPPED - window has title bar and borders
			// WS_CLIPSIBLINGS - do not render over sibling windows
			// WS_CLIPCHILDREN - exclude area, occupied by children windows
			style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
			if (m_isResizeable)
			{
				// WS_SIZEBOX = WM_THICKFRAME
				style |= WS_MAXIMIZEBOX | WS_SIZEBOX;
			}
			exStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;

			rect.left = x;
			rect.right = x + m_width;
			rect.top = y;
			rect.bottom = y + m_height;
		}

		AdjustWindowRectEx (&rect, style, FALSE, exStyle);

		wchar_t placeholderTitle[] = L"Title";
		m_hWnd = CreateWindowEx(
			exStyle,
			m_classname,
			placeholderTitle,
			style,
			rect.left,
			rect.top,
			rect.right - rect.left,
			rect.bottom - rect.top,
			NULL,
			NULL,
			m_hInstance,
			NULL
			);

		if (!m_hWnd)
		{
			// TODO: error
			printf("Failed to create window!\n");
			return false;
		}

		if (m_ownsDC)
		{
			m_hDC = GetDC(m_hWnd);
			if (m_hDC == nullptr)
			{
				// TODO: error
				printf("Failed to acquire window device context!\n");
				return false;
			}
		}

		SetWindowPos(
			m_hWnd,
			HWND_TOP,
			rect.left,
			rect.top,
			rect.right - rect.left,
			rect.bottom - rect.top,
			SWP_FRAMECHANGED
			);

		ShowWindow(m_hWnd, SW_SHOW);
		SetForegroundWindow(m_hWnd);
		SetFocus(m_hWnd);
		UpdateWindow(m_hWnd);

		GetClientRect(m_hWnd, &rect);

		m_width = rect.right - rect.left;
		m_height = rect.bottom - rect.top;

		SetCursorPos(x + m_width / 2, y + m_height / 2);

		return true;
	}

	void Window::deinit()
	{
		if (m_fullscreen)
		{
			ChangeDisplaySettings(NULL, CDS_RESET);
			ShowCursor(TRUE);
			Sleep(2);
		}

		if (m_ownsDC && m_hDC)
		{
			ReleaseDC(m_hWnd, m_hDC);
			m_hDC = nullptr;
		}

		if (m_hWnd)
		{
			DestroyWindow(m_hWnd);
			m_hWnd = nullptr;
		}

		if (m_hInstance)
			UnregisterClass(m_classname, m_hInstance);
	}

	KeyCode convertExtendedVKeyToKeyCode(WPARAM wParam, LPARAM lParam)
	{
		UINT scancode = (lParam & 0x00ff0000) >> 16;
		int extended  = (lParam & 0x01000000) != 0;

		switch (wParam)
		{
			case VK_SHIFT:
			{
				UINT mappedVKey = MapVirtualKey(scancode, MAPVK_VSC_TO_VK_EX);
				return (mappedVKey == VK_RSHIFT) ? KeyCode::eRShift : KeyCode::eLShift;
			}
			case VK_CONTROL:
			{
				return extended ? KeyCode::eRCtrl : KeyCode::eLCtrl;
			}
			case VK_MENU:
			{
				return extended ? KeyCode::eRAlt : KeyCode::eLAlt;
			}
		}
		return KeyCode::eNUM_ENTRIES;
	}

	KeyCode convertVKeyToKeyCode(WPARAM wParam, LPARAM lParam)
	{
		switch (wParam)
		{
			case VK_ESCAPE:
			{
				return KeyCode::eEscape;
			}

			case VK_SHIFT:
			case VK_CONTROL:
			case VK_MENU:
			{
				return convertExtendedVKeyToKeyCode(wParam, lParam);
			}

			case VK_RETURN: 
			{
				return KeyCode::eEnter;
			}
			case VK_SPACE: 
			{
				return KeyCode::eSpace;
			}
			case VK_TAB: 
			{
				return KeyCode::eTab;
			}

			case VK_INSERT:	{ return KeyCode::eInsert; }
			case VK_DELETE:	{ return KeyCode::eDelete; }
			case VK_HOME:	{ return KeyCode::eHome; }
			case VK_END:	{ return KeyCode::eEnd; }
			case VK_PRIOR:	{ return KeyCode::ePgUp; }
			case VK_NEXT:	{ return KeyCode::ePgDown; }

			case VK_F1:		{ return KeyCode::eF1; }
			case VK_F2:		{ return KeyCode::eF2; }
			case VK_F3:		{ return KeyCode::eF3; }
			case VK_F4:		{ return KeyCode::eF4; }
			case VK_F5:		{ return KeyCode::eF5; }
			case VK_F6:		{ return KeyCode::eF6; }
			case VK_F7:		{ return KeyCode::eF7; }
			case VK_F8:		{ return KeyCode::eF8; }
			case VK_F9:		{ return KeyCode::eF9; }
			case VK_F10:	{ return KeyCode::eF10; }
			case VK_F11:	{ return KeyCode::eF11; }
			case VK_F12:	{ return KeyCode::eF12; }

			default:
			{
				return (KeyCode)wParam;
			}
		}
	}

	LRESULT CALLBACK WindowProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
	{
		switch (msg)
		{
			case WM_KEYDOWN:
			{
				if (g_pfnKeyStateCallback)
				{
					KeyState keyState = KeyState::ePressed;
					if (((lParam >> 30) & 1) != 0)
					{
						keyState = KeyState::eHeldDown;
					}
					g_pfnKeyStateCallback(g_pUserData, convertVKeyToKeyCode(wParam, lParam), keyState);
				}
				break;
			}
			case WM_KEYUP:
			{
				if (g_pfnKeyStateCallback)
				{
					g_pfnKeyStateCallback(g_pUserData, convertVKeyToKeyCode(wParam, lParam), KeyState::eReleased);
				}
				break;
			}
			case WM_SYSKEYDOWN:
			{
				if (g_pfnKeyStateCallback)
				{
					KeyState keyState = KeyState::ePressed;
					if (((lParam >> 30) & 1) != 0)
					{
						keyState = KeyState::eHeldDown;
					}
					g_pfnKeyStateCallback(g_pUserData, convertVKeyToKeyCode(wParam, lParam), keyState);
				}
				break;
			}
			case WM_SYSKEYUP:
			{
				if (g_pfnKeyStateCallback)
				{
					g_pfnKeyStateCallback(g_pUserData, convertVKeyToKeyCode(wParam, lParam), KeyState::eReleased);
				}
				break;
			}
			case WM_SETFOCUS:
			case WM_KILLFOCUS:
			{
				if (g_pfnChangeFocusCallback)
				{
					g_pfnChangeFocusCallback(g_pUserData, (msg == WM_SETFOCUS));
				}
				return FALSE;
			}
			case WM_ACTIVATE:
			{
				//active = (LOWORD(wParam) == WA_INACTIVE);
				return FALSE;
			}
			case WM_CLOSE:
			{
				if (g_pfnCloseCallback)
				{
					g_pfnCloseCallback(g_pUserData);
				}
				PostQuitMessage(0);
				return FALSE;
			}
			case WM_SYSCOMMAND:
			{
				switch (wParam & 0xFFF0)
				{
				case SC_SCREENSAVE:
				case SC_MONITORPOWER:
					return FALSE;
					break;

				case SC_KEYMENU:
					return FALSE;
				}
				break;
			}
			case WM_SIZE:
			{
				if (g_pfnResizeCallback)
				{
					g_pfnResizeCallback(g_pUserData, LOWORD(lParam), HIWORD(lParam));
				}
				break;
			}
			case WM_ERASEBKGND:
			{
				return FALSE;
			}
		}

		return DefWindowProc(hWnd, msg, wParam, lParam);
	}
}
