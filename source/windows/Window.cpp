#include <stdio.h>

#include <map>

#include "windows/window.h"

typedef uint32_t uint;

namespace windows
{
	// WARNING: several windows supported, however they should be managed only from one thread
	std::map<HWND, Window *> g_windowMap;
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

	pfnChangeActiveCallback g_pfnChangeActiveCallback = nullptr;
	void setChangeActiveCallback(pfnChangeActiveCallback callback)
	{
		g_pfnChangeActiveCallback = callback;
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

	pfnMouseEventCallback g_pfnMouseEventCallback = nullptr;
	void setMouseEventCallback(pfnMouseEventCallback callback)
	{
		g_pfnMouseEventCallback = callback;
	}

	void getMouseCoordinates(int * mx, int * my)
	{
		if (!mx || !my)
			return;

		POINT mousePos;
		GetCursorPos(&mousePos);
		*mx = mousePos.x;
		*my = mousePos.y;
	}
	void setMouseCoordinates(int mx, int my)
	{
		SetCursorPos(mx, my);
	}

	void showMouse()
	{
		while (ShowCursor(true) < 0);
	}
	void hideMouse()
	{
		while (ShowCursor(false) >= 0);
	}

	uint Window::windowsCount = 0;

	void Window::setParameters(uint32_t width, uint32_t height, Kind windowKind)
	{
		m_width = width;
		m_height = height;
		m_windowKind = windowKind;
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

		const bool borderlessFullscreen = (m_windowKind == Kind::eFullscreenBorderless);

		int screenWidth = GetSystemMetrics(SM_CXSCREEN);
		int screenHeight = GetSystemMetrics(SM_CYSCREEN);

		if (borderlessFullscreen)
		{
			m_width = screenWidth;
			m_height = screenHeight;
		}
		x = (screenWidth - m_width)  / 2;
		y = (screenHeight - m_height) / 2;

		if (m_windowKind == Kind::eFullscreen)
		{
			DEVMODE dmScreenSettings;
			memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));
			dmScreenSettings.dmSize = sizeof(dmScreenSettings);
			dmScreenSettings.dmPelsWidth = m_width;
			dmScreenSettings.dmPelsHeight = m_height;
			dmScreenSettings.dmBitsPerPel = 32;
			dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

			if ((m_width != (uint32_t)screenWidth) && (m_height != (uint32_t)screenHeight))
			{
				if (ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN) != DISP_CHANGE_SUCCESSFUL)
				{
					// Probably makes sense to drop to fullscreen borderless here, but that will change target res
					m_windowKind = Kind::eWindowed;
					// TODO: warning
					printf("Fullscreen is not supprted!\n");
				}
			}
		}

		if (m_windowKind == Kind::eFullscreen)
		{
			// See style definitions below
			style = WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
			exStyle = WS_EX_APPWINDOW;

			rect.left = 0;
			rect.right = m_width;
			rect.top = 0;
			rect.bottom = m_height;
		}
		else
		{
			// WS_OVERLAPPEDWINDOW = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX
			// WS_CAPTION - window includes title bar (includes the WS_BORDER style)
			// WS_OVERLAPPED - window has title bar and borders
			// WS_CLIPSIBLINGS - do not render over sibling windows
			// WS_CLIPCHILDREN - exclude area, occupied by children windows
			if (borderlessFullscreen)
			{
				style = WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
			}
			else
			{
				style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_POPUP;
			}
			if (m_isResizeable && !borderlessFullscreen)
			{
				// WS_SIZEBOX = WM_THICKFRAME
				style |= WS_MAXIMIZEBOX | WS_SIZEBOX;
			}

			if (borderlessFullscreen)
			{
				exStyle = WS_EX_APPWINDOW | WS_EX_TOPMOST;
			}
			else
			{
				exStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;
			}

			rect.left = x;
			rect.right = x + m_width;
			rect.top = y;
			rect.bottom = y + m_height;
		}

		AdjustWindowRectEx (&rect, style, FALSE, exStyle);

		wchar_t placeholderTitle[] = L"Loading";
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

		// Register keyboard device for RawInput message listener

		m_riDevices.resize(0);

		{
			RAWINPUTDEVICE riKeyboard;
			riKeyboard.usUsagePage = 0x01;			// generic desktop controls
			riKeyboard.usUsage = 0x06;				// keyboard
			riKeyboard.dwFlags = 0;
			riKeyboard.hwndTarget = m_hWnd;
			m_riDevices.push_back(riKeyboard);
			m_riKeyboardIndex = (int)m_riDevices.size();
		}
		{
			RAWINPUTDEVICE riMouse;
			riMouse.usUsagePage = 0x01;			// generic desktop controls
			riMouse.usUsage = 0x02;				// mouse
			riMouse.dwFlags = 0;
			riMouse.hwndTarget = m_hWnd;
			m_riDevices.push_back(riMouse);
			m_riMouseIndex = (int)m_riDevices.size();
		}

		if (m_riDevices.size() > 0)
		{
			RegisterRawInputDevices(m_riDevices.data(), (UINT)m_riDevices.size(), sizeof(RAWINPUTDEVICE));
		}

		g_windowMap.insert(std::make_pair(m_hWnd, this));

		return true;
	}

	void Window::deinit()
	{
		for (size_t riIdx = 0, riIdxEnd = m_riDevices.size(); riIdx < riIdxEnd; ++riIdx)
		{
			m_riDevices[riIdx].dwFlags = RIDEV_REMOVE;
			m_riDevices[riIdx].hwndTarget = 0;
		}
		RegisterRawInputDevices(m_riDevices.data(), (UINT)m_riDevices.size(), sizeof(RAWINPUTDEVICE));
		m_riDevices.resize(0);

		g_windowMap.erase(m_hWnd);

		if (m_windowKind == Kind::eFullscreen)
		{
			ChangeDisplaySettings(NULL, CDS_RESET);
			showMouse();
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

	LRESULT CALLBACK WindowProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
	{
		switch (msg)
		{
			case WM_INPUT:
			{
				char buffer[sizeof(RAWINPUT)] = {};
				UINT size = sizeof(RAWINPUT);
				GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, buffer, &size, sizeof(RAWINPUTHEADER));
 
				RAWINPUT * raw = reinterpret_cast<RAWINPUT *>(buffer);
				if (raw->header.dwType == RIM_TYPEKEYBOARD && g_pfnKeyStateCallback)
				{
					const RAWKEYBOARD & rawKB = raw->data.keyboard;

					UINT virtualKey = rawKB.VKey;
					UINT scanCode = rawKB.MakeCode;
					UINT flags = rawKB.Flags;

					auto preprocessKeyData = [](UINT & virtualKey, UINT & scanCode, UINT & flags)
					{
						if (virtualKey == 255)
						{
							// Part of an escaped sequence - discard
							return 0;
						}
						else if (virtualKey == VK_SHIFT)
						{
							// Is Shift Left or Right?
							virtualKey = MapVirtualKey(scanCode, MAPVK_VSC_TO_VK_EX);
						}
						else if (virtualKey == VK_NUMLOCK)
						{
							// NumLock sends the same scanCode as Pause/Break, but different virtualKey
							//	need to resolve this
							scanCode = (MapVirtualKey(virtualKey, MAPVK_VK_TO_VSC) | 0x100);
						}
						return 1;
					};

					auto hWndPair = g_windowMap.find(hWnd);
					if (preprocessKeyData(virtualKey, scanCode, flags) && hWndPair != g_windowMap.end())
					{
						// scanCode prefixes
						const bool e0 = ((flags & RI_KEY_E0) != 0);
						const bool e1 = ((flags & RI_KEY_E1) != 0);
						const bool keyDown = ((flags & RI_KEY_MAKE) != 0);
						const bool keyUp = ((flags & RI_KEY_BREAK) != 0);

						if (e1)
						{
							if (virtualKey == VK_PAUSE)
								scanCode = 0x45;
							else
								scanCode = MapVirtualKey(virtualKey, MAPVK_VK_TO_VSC);
						}

						KeyCode keyCode;
						switch (virtualKey)
						{
							case VK_ESCAPE:		{ keyCode = KeyCode::eEscape; break; }
							case VK_LSHIFT:		{ keyCode = KeyCode::eLShift; break; }
							case VK_RSHIFT:		{ keyCode = KeyCode::eRShift; break; }
							case VK_LCONTROL:	{ keyCode = KeyCode::eLCtrl; break; }
							case VK_RCONTROL:	{ keyCode = KeyCode::eRCtrl; break; }
							case VK_LMENU:		{ keyCode = KeyCode::eLAlt; break; }
							case VK_RMENU:		{ keyCode = KeyCode::eRAlt; break; }
							case VK_RETURN:		{ keyCode = KeyCode::eEnter; break; }
							case VK_SPACE:		{ keyCode = KeyCode::eSpace; break; }
							case VK_TAB:		{ keyCode = KeyCode::eTab; break; }

							case VK_INSERT:		{ keyCode = KeyCode::eInsert; break; }
							case VK_DELETE:		{ keyCode = KeyCode::eDelete; break; }
							case VK_HOME:		{ keyCode = KeyCode::eHome; break; }
							case VK_END:		{ keyCode = KeyCode::eEnd; break; }
							case VK_PRIOR:		{ keyCode = KeyCode::ePgUp; break; }
							case VK_NEXT:		{ keyCode = KeyCode::ePgDown; break; }

							case VK_F1:			{ keyCode = KeyCode::eF1; break; }
							case VK_F2:			{ keyCode = KeyCode::eF2; break; }
							case VK_F3:			{ keyCode = KeyCode::eF3; break; }
							case VK_F4:			{ keyCode = KeyCode::eF4; break; }
							case VK_F5:			{ keyCode = KeyCode::eF5; break; }
							case VK_F6:			{ keyCode = KeyCode::eF6; break; }
							case VK_F7:			{ keyCode = KeyCode::eF7; break; }
							case VK_F8:			{ keyCode = KeyCode::eF8; break; }
							case VK_F9:			{ keyCode = KeyCode::eF9; break; }
							case VK_F10:		{ keyCode = KeyCode::eF10; break; }
							case VK_F11:		{ keyCode = KeyCode::eF11; break; }
							case VK_F12:		{ keyCode = KeyCode::eF12; break; }

							default:
							{
								keyCode = (KeyCode)virtualKey;
								break;
							}
						}

						Window * curWindow = hWndPair->second;
						KeyState keyState;
						if (keyUp)
						{
							keyState = KeyState::eReleased;
							curWindow->setKeyState((int)virtualKey, 0);
						}
						else
						{
							char curKeyState = curWindow->getKeyState((int)virtualKey);
							keyState = KeyState::ePressed;
							if (curKeyState != 0)
								keyState = KeyState::eHeldDown;
							curWindow->setKeyState((int)virtualKey, 1);
						}

						g_pfnKeyStateCallback(g_pUserData, keyCode, keyState);
					}
				}
				else if (raw->header.dwType == RIM_TYPEMOUSE && g_pfnMouseEventCallback)
				{
					// RawInput should always be relative (when not in remote desktop mode at least)
					bool absoluteCoordinates = (raw->data.mouse.usFlags & MOUSE_MOVE_ABSOLUTE) == MOUSE_MOVE_ABSOLUTE;
					if (!absoluteCoordinates)
					{
						MouseEvent mouseEvent = {};
						mouseEvent.dX = raw->data.mouse.lLastX;
						mouseEvent.dY = raw->data.mouse.lLastY;
						mouseEvent.type = MouseEvent::Type::eMove;
						if (mouseEvent.dX != 0 || mouseEvent.dY != 0)
						{
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
							mouseEvent.dX = 0;
							mouseEvent.dY = 0;
						}
						if (raw->data.mouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_DOWN)
						{
							mouseEvent.type = MouseEvent::Type::eLBDown;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
						else if (raw->data.mouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_UP)
						{
							mouseEvent.type = MouseEvent::Type::eLBUp;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
						if (raw->data.mouse.usButtonFlags & RI_MOUSE_MIDDLE_BUTTON_DOWN)
						{
							mouseEvent.type = MouseEvent::Type::eMBDown;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
						else if (raw->data.mouse.usButtonFlags & RI_MOUSE_MIDDLE_BUTTON_UP)
						{
							mouseEvent.type = MouseEvent::Type::eMBUp;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
						if (raw->data.mouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_DOWN)
						{
							mouseEvent.type = MouseEvent::Type::eRBDown;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
						else if (raw->data.mouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_UP)
						{
							mouseEvent.type = MouseEvent::Type::eRBUp;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
						if (raw->data.mouse.usButtonFlags & RI_MOUSE_WHEEL)
						{
							mouseEvent.type = MouseEvent::Type::eWheel;
							const int c_wheelStep = WHEEL_DELTA;
							mouseEvent.dX = (short)raw->data.mouse.usButtonData / c_wheelStep;
							g_pfnMouseEventCallback(g_pUserData, mouseEvent);
						}
					}
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
				if (g_pfnChangeActiveCallback)
				{
					g_pfnChangeActiveCallback(g_pUserData, (LOWORD(wParam) != WA_INACTIVE));
				}
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
