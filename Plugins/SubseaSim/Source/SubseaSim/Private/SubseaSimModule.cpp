#include "SubseaSimModule.h"

#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Sensors/SubseaDepthCameraComponent.h"

DEFINE_LOG_CATEGORY(LogSubseaSim);

IMPLEMENT_MODULE(FDefaultModuleImpl, SubseaSim);

namespace
{
	/**
	 * `Subsea.Benchmark`: times the CPU half of the depth camera (un-projection, noise model and
	 * packing) across resolutions and writes Saved/Benchmarks/unproject.csv. Needs no GPU, so it
	 * runs in CI with -nullrhi.
	 */
	void RunUnprojectBenchmarkImpl()
	{
		struct FCase
		{
			int32 Width;
			int32 Height;
		};
		const FCase Cases[] = { { 320, 240 }, { 640, 480 }, { 1280, 720 }, { 1920, 1080 } };
		constexpr int32 Iterations = 30;

		FString Csv = TEXT("width,height,pixels,points,mean_ms,min_ms,mpoints_per_s\n");
		UE_LOG(LogSubseaSim, Display, TEXT("Subsea.Benchmark: depth camera un-projection, %d iterations per case"), Iterations);

		for (const FCase& Case : Cases)
		{
			// A sloping seabed between 1 m and 9 m so every pixel is in range and does real work.
			TArray<FLinearColor> Pixels;
			Pixels.SetNumUninitialized(Case.Width * Case.Height);
			for (int32 Row = 0; Row < Case.Height; ++Row)
			{
				const float Depth = 100.0f + 800.0f * static_cast<float>(Row) / static_cast<float>(Case.Height);
				for (int32 Col = 0; Col < Case.Width; ++Col)
				{
					Pixels[Row * Case.Width + Col] = FLinearColor(0.1f, 0.3f, 0.4f, Depth);
				}
			}

			FSubseaRangeNoiseModel Noise;
			TArray<Subsea::Protocol::FPackedPoint> Points;
			double Total = 0.0;
			double Fastest = TNumericLimits<double>::Max();
			for (int32 Iteration = 0; Iteration < Iterations + 2; ++Iteration)
			{
				const double Start = FPlatformTime::Seconds();
				USubseaDepthCameraComponent::Unproject(Pixels, Case.Width, Case.Height, 90.0f, Noise, 0.15, 1.0f, Iteration, Points);
				const double Elapsed = (FPlatformTime::Seconds() - Start) * 1000.0;
				if (Iteration >= 2) // discard warm-up
				{
					Total += Elapsed;
					Fastest = FMath::Min(Fastest, Elapsed);
				}
			}

			const double Mean = Total / Iterations;
			const double Rate = static_cast<double>(Case.Width) * Case.Height / (Mean * 1000.0);
			UE_LOG(LogSubseaSim, Display, TEXT("  %4dx%-4d  %8d px  %8d pts  mean %6.2f ms  min %6.2f ms  %7.1f Mpx/s"),
				Case.Width, Case.Height, Case.Width * Case.Height, Points.Num(), Mean, Fastest, Rate);
			Csv += FString::Printf(TEXT("%d,%d,%d,%d,%.4f,%.4f,%.2f\n"), Case.Width, Case.Height, Case.Width * Case.Height, Points.Num(), Mean, Fastest, Rate);
		}

		const FString Path = FPaths::ProjectSavedDir() / TEXT("Benchmarks/unproject.csv");
		if (FFileHelper::SaveStringToFile(Csv, *Path))
		{
			UE_LOG(LogSubseaSim, Display, TEXT("Subsea.Benchmark: wrote %s"), *Path);
		}
	}

	FAutoConsoleCommand BenchmarkCommand(
		TEXT("Subsea.Benchmark"),
		TEXT("Times depth camera un-projection across resolutions and writes Saved/Benchmarks/unproject.csv."),
		FConsoleCommandDelegate::CreateStatic(&RunUnprojectBenchmarkImpl));
}

void Subsea::RunUnprojectBenchmark()
{
	RunUnprojectBenchmarkImpl();
}
