#ifndef __HELP_H__
#define __HELP_H__

#include <iostream>

template<typename T>
class AutoFree
{
public:
	AutoFree(T** t) :data(t)
	{

	}
	~AutoFree()
	{
		delete *data;
	}
private:
	T** data;
};

#define AUTO_FREE(type,pointer,id) AutoFree<##type> auto_##id(pointer);

#endif // !__HELP_H__
