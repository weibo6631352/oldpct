#pragma once
#include <windows.h>
#include <stdio.h>

static void __cdecl  dprint(const char* fmt, ...)
{
	char buf[1024] = { 0 }, *p = buf;
	va_list args;

	char fmtfmt[256] = { 0 };
	sprintf_s(fmtfmt, "HYDBG\t%s", fmt);

	va_start(args, fmt);
	p += vsnprintf_s(p, sizeof(buf), _TRUNCATE, fmtfmt, args);
	va_end(args);

// 	while (p > buf  &&  isspace(p[-1]))
// 		*--p = '\0';
// 	*p++ = '\r';
// 	*p++ = '\n';
// 	*p = '\0';

	OutputDebugStringA(buf);    //OutputDebugString    
}

#define dd(Format, ...)  dprint(Format, __VA_ARGS__)