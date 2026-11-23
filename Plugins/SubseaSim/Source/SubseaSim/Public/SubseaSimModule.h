#pragma once

#include "CoreMinimal.h"
#include "Stats/Stats.h"

SUBSEASIM_API DECLARE_LOG_CATEGORY_EXTERN(LogSubseaSim, Log, All);

DECLARE_STATS_GROUP(TEXT("SubseaSim"), STATGROUP_SubseaSim, STATCAT_Advanced);

namespace Subsea
{
	/** Runs the depth camera un-projection benchmark and writes Saved/Benchmarks/unproject.csv. */
	SUBSEASIM_API void RunUnprojectBenchmark();
}
