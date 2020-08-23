#pragma once

#ifdef BETA_EXPORTS
#define BETA_API __declspec(dllexport)
#else
#define BETA_API __declspec(dllexport)
#endif


extern "C"
{

	struct Vector3 // THIS IS A "Vector3" AS UNITY KNOWS IT
	{
		float x;
		float y;
		float z;
	};
	int BETA_API dummy_function(int handle);
	
	int BETA_API Init(int handle);

	float BETA_API MonoUpdate(int handle);
	
	float BETA_API *ImageUpdate(int handle);

	void BETA_API Shutdown(int handle);
	
}
