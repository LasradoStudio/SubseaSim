#include "Editor.h"
#include "EngineUtils.h"
#include "Framework/Notifications/NotificationManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "ScopedTransaction.h"
#include "Sensors/SubseaDepthCameraComponent.h"
#include "ToolMenus.h"
#include "Vehicle/SubseaROV.h"
#include "Widgets/Notifications/SNotificationList.h"
#include "World/SubseaEnvironment.h"

#define LOCTEXT_NAMESPACE "SubseaSimEditor"

/**
 * Editor tooling for scenario authors: a "Subsea Sim" section in the Tools menu that drops a
 * ready-made inspection site into the open level and pulls sensor data out of a running session.
 */
class FSubseaSimEditorModule final : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FSubseaSimEditorModule::RegisterMenus));
	}

	virtual void ShutdownModule() override
	{
		UToolMenus::UnRegisterStartupCallback(this);
		UToolMenus::UnregisterOwner(this);
	}

private:
	void RegisterMenus()
	{
		FToolMenuOwnerScoped OwnerScoped(this);
		UToolMenu* Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Tools");
		FToolMenuSection& Section = Menu->AddSection("SubseaSim", LOCTEXT("Section", "Subsea Sim"));

		Section.AddMenuEntry("SubseaAddSite",
			LOCTEXT("AddSite", "Add Inspection Site to Level"),
			LOCTEXT("AddSiteTip", "Spawns a procedural seabed, pipeline and manifold, plus an ROV at the start of the pipeline."),
			FSlateIcon(), FUIAction(FExecuteAction::CreateStatic(&FSubseaSimEditorModule::AddSite)));

		Section.AddMenuEntry("SubseaExportCloud",
			LOCTEXT("ExportCloud", "Export Depth Camera Cloud (.ply)"),
			LOCTEXT("ExportCloudTip", "While playing in the editor, writes the ROV's latest point cloud to Saved/PointClouds."),
			FSlateIcon(), FUIAction(FExecuteAction::CreateStatic(&FSubseaSimEditorModule::ExportCloud)));
	}

	static void Notify(const FText& Message, const bool bSuccess)
	{
		FNotificationInfo Info(Message);
		Info.ExpireDuration = 5.0f;
		const TSharedPtr<SNotificationItem> Item = FSlateNotificationManager::Get().AddNotification(Info);
		if (Item.IsValid())
		{
			Item->SetCompletionState(bSuccess ? SNotificationItem::CS_Success : SNotificationItem::CS_Fail);
		}
	}

	static void AddSite()
	{
		UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World)
		{
			return;
		}
		for (TActorIterator<ASubseaEnvironment> It(World); It; ++It)
		{
			Notify(LOCTEXT("SiteExists", "This level already has a subsea inspection site."), false);
			return;
		}

		const FScopedTransaction Transaction(LOCTEXT("AddSiteTransaction", "Add Subsea Inspection Site"));
		ASubseaEnvironment* Environment = World->SpawnActor<ASubseaEnvironment>(FVector::ZeroVector, FRotator::ZeroRotator);
		if (!Environment)
		{
			return;
		}
		const FTransform Start = Environment->GetVehicleStart();
		World->SpawnActor<ASubseaROV>(Start.GetLocation(), Start.Rotator());
		GEditor->SelectNone(false, true);
		GEditor->SelectActor(Environment, true, true);
		Notify(LOCTEXT("SiteAdded", "Added a subsea inspection site and ROV."), true);
	}

	static void ExportCloud()
	{
		UWorld* PlayWorld = GEditor ? GEditor->PlayWorld.Get() : nullptr;
		if (!PlayWorld)
		{
			Notify(LOCTEXT("NeedPie", "Start Play In Editor first: the depth camera only runs while simulating."), false);
			return;
		}
		for (TActorIterator<ASubseaROV> It(PlayWorld); It; ++It)
		{
			const FString Path = FPaths::ProjectSavedDir() / TEXT("PointClouds") / FString::Printf(TEXT("cloud_%s.ply"), *FDateTime::Now().ToString());
			const bool bSaved = It->GetDepthCamera() && It->GetDepthCamera()->ExportLatestCloudPly(Path);
			Notify(bSaved
				? FText::Format(LOCTEXT("Exported", "Saved {0}"), FText::FromString(Path))
				: LOCTEXT("ExportFailed", "No point cloud available yet."), bSaved);
			return;
		}
		Notify(LOCTEXT("NoRov", "No ROV in the running level."), false);
	}
};

IMPLEMENT_MODULE(FSubseaSimEditorModule, SubseaSimEditor)

#undef LOCTEXT_NAMESPACE
