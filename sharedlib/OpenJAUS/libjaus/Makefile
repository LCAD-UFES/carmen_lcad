TARGET	= 	./lib/libjaus.so
CCFLAGS	=	-Wall -std=c99 -fPIC -fno-strict-aliasing -O2 -c -g -I ./include/
SOURCES =	./src/jausComponent.c \
		./src/jausAddress.c \
		./src/jausNode.c \
		./src/type/jausShort.c \
		./src/type/jausWorldModelVectorObject.c \
		./src/type/jausTime.c \
		./src/type/jausWorldModelFeatureClass.c \
		./src/type/jausInteger.c \
		./src/type/jausMissionTask.c \
		./src/type/jausEventLimit.c \
		./src/type/jausGeometryPointXYZ.c \
		./src/type/jausUnsignedInteger.c \
		./src/type/jausByte.c \
		./src/type/jausFloat.c \
		./src/type/jausGeometryPointLLA.c \
		./src/type/jausUnsignedShort.c \
		./src/type/jausBoolean.c \
		./src/type/jausLong.c \
		./src/type/jausUnsignedLong.c \
		./src/type/jausDouble.c \
		./src/type/jausMissionCommand.c \
		./src/jausArray.c \
		./src/jausState.c \
		./src/jausService.c \
		./src/jausPayloadInterface.c \
		./src/message/query/communications/queryDataLinkStatusMessage.c \
		./src/message/query/communications/queryHeartbeatPulseMessage.c \
		./src/message/query/communications/querySelectedDataLinkStatusMessage.c \
		./src/message/query/dynamicConfiguration/querySubsystemListMessage.c \
		./src/message/query/dynamicConfiguration/queryIdentificationMessage.c \
		./src/message/query/dynamicConfiguration/queryConfigurationMessage.c \
		./src/message/query/dynamicConfiguration/queryServicesMessage.c \
		./src/message/query/manipulator/queryJointForceTorquesMessage.c \
		./src/message/query/manipulator/queryJointVelocitiesMessage.c \
		./src/message/query/manipulator/queryJointPositionsMessage.c \
		./src/message/query/manipulator/queryManipulatorSpecificationsMessage.c \
		./src/message/query/manipulator/queryJointEffortsMessage.c \
		./src/message/query/manipulator/queryToolPointMessage.c \
		./src/message/query/core/queryComponentAuthorityMessage.c \
		./src/message/query/core/queryComponentStatusMessage.c \
		./src/message/query/core/queryComponentControlMessage.c \
		./src/message/query/core/queryTimeMessage.c \
		./src/message/query/platform/queryLocalWaypointMessage.c \
		./src/message/query/platform/queryDiscreteDevicesMessage.c \
		./src/message/query/platform/queryPathSegmentCountMessage.c \
		./src/message/query/platform/queryPlatformOperationalDataMessage.c \
		./src/message/query/platform/queryPlatformSpecificationsMessage.c \
		./src/message/query/platform/queryLocalVectorMessage.c \
		./src/message/query/platform/queryWaypointCountMessage.c \
		./src/message/query/platform/queryWrenchEffortMessage.c \
		./src/message/query/platform/queryGlobalWaypointMessage.c \
		./src/message/query/platform/queryGlobalPathSegmentMessage.c \
		./src/message/query/platform/queryTravelSpeedMessage.c \
		./src/message/query/platform/queryLocalPathSegmentMessage.c \
		./src/message/query/platform/queryGlobalPoseMessage.c \
		./src/message/query/platform/queryLocalPoseMessage.c \
		./src/message/query/platform/queryGlobalVectorMessage.c \
		./src/message/query/platform/queryVelocityStateMessage.c \
		./src/message/query/payload/queryPayloadDataElementMessage.c \
		./src/message/query/payload/queryPayloadInterfaceMessage.c \
		./src/message/query/event/queryEventsMessage.c \
		./src/message/query/planning/querySpoolingPreferenceMessage.c \
		./src/message/query/planning/queryMissionStatusMessage.c \
		./src/message/query/worldModel/queryVksObjectsMessage.c \
		./src/message/query/worldModel/queryVksBoundsMessage.c \
		./src/message/query/worldModel/queryVksFeatureClassMetadataMessage.c \
		./src/message/query/environment/queryCameraFormatOptionsMessage.c \
		./src/message/query/environment/queryImageMessage.c \
		./src/message/query/environment/queryCameraPoseMessage.c \
		./src/message/query/environment/queryCameraCapabilitiesMessage.c \
		./src/message/query/environment/queryCameraCountMessage.c \
		./src/message/query/environment/queryRelativeObjectPositionMessage.c \
		./src/message/query/environment/querySelectedCameraMessage.c \
		./src/message/inform/communications/reportSelectedDataLinkStatusMessage.c \
		./src/message/inform/communications/reportHeartbeatPulseMessage.c \
		./src/message/inform/communications/reportDataLinkStatusMessage.c \
		./src/message/inform/dynamicConfiguration/reportServicesMessage.c \
		./src/message/inform/dynamicConfiguration/reportConfigurationMessage.c \
		./src/message/inform/dynamicConfiguration/reportSubsystemListMessage.c \
		./src/message/inform/dynamicConfiguration/reportIdentificationMessage.c \
		./src/message/inform/manipulator/reportToolPointMessage.c \
		./src/message/inform/manipulator/reportJointEffortsMessage.c \
		./src/message/inform/manipulator/reportManipulatorSpecificationsMessage.c \
		./src/message/inform/manipulator/reportJointVelocitiesMessage.c \
		./src/message/inform/manipulator/reportJointForceTorquesMessage.c \
		./src/message/inform/manipulator/reportJointPositionsMessage.c \
		./src/message/inform/core/reportComponentControlMessage.c \
		./src/message/inform/core/reportComponentStatusMessage.c \
		./src/message/inform/core/reportTimeMessage.c \
		./src/message/inform/core/reportComponentAuthorityMessage.c \
		./src/message/inform/platform/reportLocalPoseMessage.c \
		./src/message/inform/platform/reportGlobalWaypointMessage.c \
		./src/message/inform/platform/reportWrenchEffortMessage.c \
		./src/message/inform/platform/reportGlobalVectorMessage.c \
		./src/message/inform/platform/reportLocalWaypointMessage.c \
		./src/message/inform/platform/reportWaypointCountMessage.c \
		./src/message/inform/platform/reportDiscreteDevicesMessage.c \
		./src/message/inform/platform/reportPlatformSpecificationsMessage.c \
		./src/message/inform/platform/reportPlatformOperationalDataMessage.c \
		./src/message/inform/platform/reportGlobalPoseMessage.c \
		./src/message/inform/platform/reportGlobalPathSegmentMessage.c \
		./src/message/inform/platform/reportTravelSpeedMessage.c \
		./src/message/inform/platform/reportLocalPathSegmentMessage.c \
		./src/message/inform/platform/reportLocalVectorMessage.c \
		./src/message/inform/platform/reportVelocityStateMessage.c \
		./src/message/inform/platform/reportPathSegmentCountMessage.c \
		./src/message/inform/payload/reportPayloadDataElementMessage.c \
		./src/message/inform/payload/reportPayloadInterfaceMessage.c \
		./src/message/inform/event/eventMessage.c \
		./src/message/inform/event/reportEventsMessage.c \
		./src/message/inform/planning/reportMissionStatusMessage.c \
		./src/message/inform/planning/reportSpoolingPreferenceMessage.c \
		./src/message/inform/worldModel/reportVksDataTransferTerminationMessage.c \
		./src/message/inform/worldModel/reportVksFeatureClassMetadataMessage.c \
		./src/message/inform/worldModel/reportVksObjectsCreationMessage.c \
		./src/message/inform/worldModel/reportVksBoundsMessage.c \
		./src/message/inform/worldModel/reportVksObjectsMessage.c \
		./src/message/inform/environment/reportImageMessage.c \
		./src/message/inform/environment/reportCameraFormatOptionsMessage.c \
		./src/message/inform/environment/reportSelectedCameraMessage.c \
		./src/message/inform/environment/reportCameraCapabilitiesMessage.c \
		./src/message/inform/environment/reportCameraCountMessage.c \
		./src/message/inform/environment/reportRelativeObjectPositionMessage.c \
		./src/message/inform/environment/reportCameraPoseMessage.c \
		./src/message/jausMessage.c \
		./src/message/command/communications/setDataLinkStatusMessage.c \
		./src/message/command/communications/setSelectedDataLinkStateMessage.c \
		./src/message/command/communications/setDataLinkSelectMessage.c \
		./src/message/command/manipulator/setJointPositionsMessage.c \
		./src/message/command/manipulator/setEndEffectorPoseMessage.c \
		./src/message/command/manipulator/setJointMotionMessage.c \
		./src/message/command/manipulator/setToolPointMessage.c \
		./src/message/command/manipulator/setEndEffectorVelocityStateMessage.c \
		./src/message/command/manipulator/setJointVelocitiesMessage.c \
		./src/message/command/manipulator/setEndEffectorPathMotionMessage.c \
		./src/message/command/manipulator/setJointEffortsMessage.c \
		./src/message/command/core/setEmergencyMessage.c \
		./src/message/command/core/standbyMessage.c \
		./src/message/command/core/confirmServiceConnectionMessage.c \
		./src/message/command/core/releaseComponentControlMessage.c \
		./src/message/command/core/suspendServiceConnectionMessage.c \
		./src/message/command/core/resumeMessage.c \
		./src/message/command/core/setTimeMessage.c \
		./src/message/command/core/confirmComponentControlMessage.c \
		./src/message/command/core/shutdownMessage.c \
		./src/message/command/core/requestComponentControlMessage.c \
		./src/message/command/core/setComponentAuthorityMessage.c \
		./src/message/command/core/resetMessage.c \
		./src/message/command/core/rejectComponentControlMessage.c \
		./src/message/command/core/terminateServiceConnectionMessage.c \
		./src/message/command/core/activateServiceConnectionMessage.c \
		./src/message/command/core/clearEmergencyMessage.c \
		./src/message/command/core/createServiceConnectionMessage.c \
		./src/message/command/platform/setTravelSpeedMessage.c \
		./src/message/command/platform/setGlobalPathSegmentMessage.c \
		./src/message/command/platform/setWrenchEffortMessage.c \
		./src/message/command/platform/setGlobalVectorMessage.c \
		./src/message/command/platform/setDiscreteDevicesMessage.c \
		./src/message/command/platform/setLocalVectorMessage.c \
		./src/message/command/platform/setVelocityStateMessage.c \
		./src/message/command/platform/setLocalPathSegmentMessage.c \
		./src/message/command/platform/setLocalWaypointMessage.c \
		./src/message/command/platform/setGlobalWaypointMessage.c \
		./src/message/command/payload/setPayloadDataElementMessage.c \
		./src/message/command/event/createEventMessage.c \
		./src/message/command/event/confirmEventRequestMessage.c \
		./src/message/command/event/cancelEventMessage.c \
		./src/message/command/event/updateEventMessage.c \
		./src/message/command/event/rejectEventRequestMessage.c \
		./src/message/command/planning/pauseMissionMessage.c \
		./src/message/command/planning/spoolMissionMessage.c \
		./src/message/command/planning/removeMessagesMessage.c \
		./src/message/command/planning/runMissionMessage.c \
		./src/message/command/planning/resumeMissionMessage.c \
		./src/message/command/planning/replaceMessagesMessage.c \
		./src/message/command/planning/abortMissionMessage.c \
		./src/message/command/worldModel/terminateVksDataTransferMessage.c \
		./src/message/command/worldModel/setVksFeatureClassMetadataMessage.c \
		./src/message/command/worldModel/createVksObjectsMessage.c \
		./src/message/command/worldModel/deleteVksObjectsMessage.c \
		./src/message/command/environment/setCameraFormatOptionsMessage.c \
		./src/message/command/environment/setCameraCapabilitiesMessage.c \
		./src/message/command/environment/setCameraPoseMessage.c \
		./src/message/command/environment/selectCameraMessage.c \
		./src/jausSubsystem.c

OBJECTS =	./Build/jausComponent.o \
		./Build/jausAddress.o \
		./Build/jausNode.o \
		./Build/jausShort.o \
		./Build/jausWorldModelVectorObject.o \
		./Build/jausTime.o \
		./Build/jausWorldModelFeatureClass.o \
		./Build/jausInteger.o \
		./Build/jausMissionTask.o \
		./Build/jausEventLimit.o \
		./Build/jausGeometryPointXYZ.o \
		./Build/jausUnsignedInteger.o \
		./Build/jausByte.o \
		./Build/jausFloat.o \
		./Build/jausGeometryPointLLA.o \
		./Build/jausUnsignedShort.o \
		./Build/jausBoolean.o \
		./Build/jausLong.o \
		./Build/jausUnsignedLong.o \
		./Build/jausDouble.o \
		./Build/jausMissionCommand.o \
		./Build/jausArray.o \
		./Build/jausState.o \
		./Build/jausService.o \
		./Build/jausPayloadInterface.o \
		./Build/queryDataLinkStatusMessage.o \
		./Build/queryHeartbeatPulseMessage.o \
		./Build/querySelectedDataLinkStatusMessage.o \
		./Build/querySubsystemListMessage.o \
		./Build/queryIdentificationMessage.o \
		./Build/queryConfigurationMessage.o \
		./Build/queryServicesMessage.o \
		./Build/queryJointForceTorquesMessage.o \
		./Build/queryJointVelocitiesMessage.o \
		./Build/queryJointPositionsMessage.o \
		./Build/queryManipulatorSpecificationsMessage.o \
		./Build/queryJointEffortsMessage.o \
		./Build/queryToolPointMessage.o \
		./Build/queryComponentAuthorityMessage.o \
		./Build/queryComponentStatusMessage.o \
		./Build/queryComponentControlMessage.o \
		./Build/queryTimeMessage.o \
		./Build/queryLocalWaypointMessage.o \
		./Build/queryDiscreteDevicesMessage.o \
		./Build/queryPathSegmentCountMessage.o \
		./Build/queryPlatformOperationalDataMessage.o \
		./Build/queryPlatformSpecificationsMessage.o \
		./Build/queryLocalVectorMessage.o \
		./Build/queryWaypointCountMessage.o \
		./Build/queryWrenchEffortMessage.o \
		./Build/queryGlobalWaypointMessage.o \
		./Build/queryGlobalPathSegmentMessage.o \
		./Build/queryTravelSpeedMessage.o \
		./Build/queryLocalPathSegmentMessage.o \
		./Build/queryGlobalPoseMessage.o \
		./Build/queryLocalPoseMessage.o \
		./Build/queryGlobalVectorMessage.o \
		./Build/queryVelocityStateMessage.o \
		./Build/queryPayloadDataElementMessage.o \
		./Build/queryPayloadInterfaceMessage.o \
		./Build/queryEventsMessage.o \
		./Build/querySpoolingPreferenceMessage.o \
		./Build/queryMissionStatusMessage.o \
		./Build/queryVksObjectsMessage.o \
		./Build/queryVksBoundsMessage.o \
		./Build/queryVksFeatureClassMetadataMessage.o \
		./Build/queryCameraFormatOptionsMessage.o \
		./Build/queryImageMessage.o \
		./Build/queryCameraPoseMessage.o \
		./Build/queryCameraCapabilitiesMessage.o \
		./Build/queryCameraCountMessage.o \
		./Build/queryRelativeObjectPositionMessage.o \
		./Build/querySelectedCameraMessage.o \
		./Build/reportSelectedDataLinkStatusMessage.o \
		./Build/reportHeartbeatPulseMessage.o \
		./Build/reportDataLinkStatusMessage.o \
		./Build/reportServicesMessage.o \
		./Build/reportConfigurationMessage.o \
		./Build/reportSubsystemListMessage.o \
		./Build/reportIdentificationMessage.o \
		./Build/reportToolPointMessage.o \
		./Build/reportJointEffortsMessage.o \
		./Build/reportManipulatorSpecificationsMessage.o \
		./Build/reportJointVelocitiesMessage.o \
		./Build/reportJointForceTorquesMessage.o \
		./Build/reportJointPositionsMessage.o \
		./Build/reportComponentControlMessage.o \
		./Build/reportComponentStatusMessage.o \
		./Build/reportTimeMessage.o \
		./Build/reportComponentAuthorityMessage.o \
		./Build/reportLocalPoseMessage.o \
		./Build/reportGlobalWaypointMessage.o \
		./Build/reportWrenchEffortMessage.o \
		./Build/reportGlobalVectorMessage.o \
		./Build/reportLocalWaypointMessage.o \
		./Build/reportWaypointCountMessage.o \
		./Build/reportDiscreteDevicesMessage.o \
		./Build/reportPlatformSpecificationsMessage.o \
		./Build/reportPlatformOperationalDataMessage.o \
		./Build/reportGlobalPoseMessage.o \
		./Build/reportGlobalPathSegmentMessage.o \
		./Build/reportTravelSpeedMessage.o \
		./Build/reportLocalPathSegmentMessage.o \
		./Build/reportLocalVectorMessage.o \
		./Build/reportVelocityStateMessage.o \
		./Build/reportPathSegmentCountMessage.o \
		./Build/reportPayloadDataElementMessage.o \
		./Build/reportPayloadInterfaceMessage.o \
		./Build/eventMessage.o \
		./Build/reportEventsMessage.o \
		./Build/reportMissionStatusMessage.o \
		./Build/reportSpoolingPreferenceMessage.o \
		./Build/reportVksDataTransferTerminationMessage.o \
		./Build/reportVksFeatureClassMetadataMessage.o \
		./Build/reportVksObjectsCreationMessage.o \
		./Build/reportVksBoundsMessage.o \
		./Build/reportVksObjectsMessage.o \
		./Build/reportImageMessage.o \
		./Build/reportCameraFormatOptionsMessage.o \
		./Build/reportSelectedCameraMessage.o \
		./Build/reportCameraCapabilitiesMessage.o \
		./Build/reportCameraCountMessage.o \
		./Build/reportRelativeObjectPositionMessage.o \
		./Build/reportCameraPoseMessage.o \
		./Build/jausMessage.o \
		./Build/setDataLinkStatusMessage.o \
		./Build/setSelectedDataLinkStateMessage.o \
		./Build/setDataLinkSelectMessage.o \
		./Build/setJointPositionsMessage.o \
		./Build/setEndEffectorPoseMessage.o \
		./Build/setJointMotionMessage.o \
		./Build/setToolPointMessage.o \
		./Build/setEndEffectorVelocityStateMessage.o \
		./Build/setJointVelocitiesMessage.o \
		./Build/setEndEffectorPathMotionMessage.o \
		./Build/setJointEffortsMessage.o \
		./Build/setEmergencyMessage.o \
		./Build/standbyMessage.o \
		./Build/confirmServiceConnectionMessage.o \
		./Build/releaseComponentControlMessage.o \
		./Build/suspendServiceConnectionMessage.o \
		./Build/resumeMessage.o \
		./Build/setTimeMessage.o \
		./Build/confirmComponentControlMessage.o \
		./Build/shutdownMessage.o \
		./Build/requestComponentControlMessage.o \
		./Build/setComponentAuthorityMessage.o \
		./Build/resetMessage.o \
		./Build/rejectComponentControlMessage.o \
		./Build/terminateServiceConnectionMessage.o \
		./Build/activateServiceConnectionMessage.o \
		./Build/clearEmergencyMessage.o \
		./Build/createServiceConnectionMessage.o \
		./Build/setTravelSpeedMessage.o \
		./Build/setGlobalPathSegmentMessage.o \
		./Build/setWrenchEffortMessage.o \
		./Build/setGlobalVectorMessage.o \
		./Build/setDiscreteDevicesMessage.o \
		./Build/setLocalVectorMessage.o \
		./Build/setVelocityStateMessage.o \
		./Build/setLocalPathSegmentMessage.o \
		./Build/setLocalWaypointMessage.o \
		./Build/setGlobalWaypointMessage.o \
		./Build/setPayloadDataElementMessage.o \
		./Build/createEventMessage.o \
		./Build/confirmEventRequestMessage.o \
		./Build/cancelEventMessage.o \
		./Build/updateEventMessage.o \
		./Build/rejectEventRequestMessage.o \
		./Build/pauseMissionMessage.o \
		./Build/spoolMissionMessage.o \
		./Build/removeMessagesMessage.o \
		./Build/runMissionMessage.o \
		./Build/resumeMissionMessage.o \
		./Build/replaceMessagesMessage.o \
		./Build/abortMissionMessage.o \
		./Build/terminateVksDataTransferMessage.o \
		./Build/setVksFeatureClassMetadataMessage.o \
		./Build/createVksObjectsMessage.o \
		./Build/deleteVksObjectsMessage.o \
		./Build/setCameraFormatOptionsMessage.o \
		./Build/setCameraCapabilitiesMessage.o \
		./Build/setCameraPoseMessage.o \
		./Build/selectCameraMessage.o \
		./Build/jausSubsystem.o

default : all

all : $(TARGET)

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)

$(TARGET) : $(OBJECTS)
	gcc -shared -o $(TARGET) $(OBJECTS)


./Build/jausComponent.o : ./src/jausComponent.c
	gcc $(CCFLAGS) -o ./Build/jausComponent.o ./src/jausComponent.c

./Build/jausAddress.o : ./src/jausAddress.c
	gcc $(CCFLAGS) -o ./Build/jausAddress.o ./src/jausAddress.c

./Build/jausNode.o : ./src/jausNode.c
	gcc $(CCFLAGS) -o ./Build/jausNode.o ./src/jausNode.c

./Build/jausShort.o : ./src/type/jausShort.c
	gcc $(CCFLAGS) -o ./Build/jausShort.o ./src/type/jausShort.c

./Build/jausWorldModelVectorObject.o : ./src/type/jausWorldModelVectorObject.c
	gcc $(CCFLAGS) -o ./Build/jausWorldModelVectorObject.o ./src/type/jausWorldModelVectorObject.c

./Build/jausTime.o : ./src/type/jausTime.c
	gcc $(CCFLAGS) -o ./Build/jausTime.o ./src/type/jausTime.c

./Build/jausWorldModelFeatureClass.o : ./src/type/jausWorldModelFeatureClass.c
	gcc $(CCFLAGS) -o ./Build/jausWorldModelFeatureClass.o ./src/type/jausWorldModelFeatureClass.c

./Build/jausInteger.o : ./src/type/jausInteger.c
	gcc $(CCFLAGS) -o ./Build/jausInteger.o ./src/type/jausInteger.c

./Build/jausMissionTask.o : ./src/type/jausMissionTask.c
	gcc $(CCFLAGS) -o ./Build/jausMissionTask.o ./src/type/jausMissionTask.c

./Build/jausEventLimit.o : ./src/type/jausEventLimit.c
	gcc $(CCFLAGS) -o ./Build/jausEventLimit.o ./src/type/jausEventLimit.c

./Build/jausGeometryPointXYZ.o : ./src/type/jausGeometryPointXYZ.c
	gcc $(CCFLAGS) -o ./Build/jausGeometryPointXYZ.o ./src/type/jausGeometryPointXYZ.c

./Build/jausUnsignedInteger.o : ./src/type/jausUnsignedInteger.c
	gcc $(CCFLAGS) -o ./Build/jausUnsignedInteger.o ./src/type/jausUnsignedInteger.c

./Build/jausByte.o : ./src/type/jausByte.c
	gcc $(CCFLAGS) -o ./Build/jausByte.o ./src/type/jausByte.c

./Build/jausFloat.o : ./src/type/jausFloat.c
	gcc $(CCFLAGS) -o ./Build/jausFloat.o ./src/type/jausFloat.c

./Build/jausGeometryPointLLA.o : ./src/type/jausGeometryPointLLA.c
	gcc $(CCFLAGS) -o ./Build/jausGeometryPointLLA.o ./src/type/jausGeometryPointLLA.c

./Build/jausUnsignedShort.o : ./src/type/jausUnsignedShort.c
	gcc $(CCFLAGS) -o ./Build/jausUnsignedShort.o ./src/type/jausUnsignedShort.c

./Build/jausBoolean.o : ./src/type/jausBoolean.c
	gcc $(CCFLAGS) -o ./Build/jausBoolean.o ./src/type/jausBoolean.c

./Build/jausLong.o : ./src/type/jausLong.c
	gcc $(CCFLAGS) -o ./Build/jausLong.o ./src/type/jausLong.c

./Build/jausUnsignedLong.o : ./src/type/jausUnsignedLong.c
	gcc $(CCFLAGS) -o ./Build/jausUnsignedLong.o ./src/type/jausUnsignedLong.c

./Build/jausDouble.o : ./src/type/jausDouble.c
	gcc $(CCFLAGS) -o ./Build/jausDouble.o ./src/type/jausDouble.c

./Build/jausMissionCommand.o : ./src/type/jausMissionCommand.c
	gcc $(CCFLAGS) -o ./Build/jausMissionCommand.o ./src/type/jausMissionCommand.c

./Build/jausArray.o : ./src/jausArray.c
	gcc $(CCFLAGS) -o ./Build/jausArray.o ./src/jausArray.c

./Build/jausState.o : ./src/jausState.c
	gcc $(CCFLAGS) -o ./Build/jausState.o ./src/jausState.c

./Build/jausService.o : ./src/jausService.c
	gcc $(CCFLAGS) -o ./Build/jausService.o ./src/jausService.c

./Build/jausPayloadInterface.o : ./src/jausPayloadInterface.c
	gcc $(CCFLAGS) -o ./Build/jausPayloadInterface.o ./src/jausPayloadInterface.c

./Build/queryDataLinkStatusMessage.o : ./src/message/query/communications/queryDataLinkStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/queryDataLinkStatusMessage.o ./src/message/query/communications/queryDataLinkStatusMessage.c

./Build/queryHeartbeatPulseMessage.o : ./src/message/query/communications/queryHeartbeatPulseMessage.c
	gcc $(CCFLAGS) -o ./Build/queryHeartbeatPulseMessage.o ./src/message/query/communications/queryHeartbeatPulseMessage.c

./Build/querySelectedDataLinkStatusMessage.o : ./src/message/query/communications/querySelectedDataLinkStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/querySelectedDataLinkStatusMessage.o ./src/message/query/communications/querySelectedDataLinkStatusMessage.c

./Build/querySubsystemListMessage.o : ./src/message/query/dynamicConfiguration/querySubsystemListMessage.c
	gcc $(CCFLAGS) -o ./Build/querySubsystemListMessage.o ./src/message/query/dynamicConfiguration/querySubsystemListMessage.c

./Build/queryIdentificationMessage.o : ./src/message/query/dynamicConfiguration/queryIdentificationMessage.c
	gcc $(CCFLAGS) -o ./Build/queryIdentificationMessage.o ./src/message/query/dynamicConfiguration/queryIdentificationMessage.c

./Build/queryConfigurationMessage.o : ./src/message/query/dynamicConfiguration/queryConfigurationMessage.c
	gcc $(CCFLAGS) -o ./Build/queryConfigurationMessage.o ./src/message/query/dynamicConfiguration/queryConfigurationMessage.c

./Build/queryServicesMessage.o : ./src/message/query/dynamicConfiguration/queryServicesMessage.c
	gcc $(CCFLAGS) -o ./Build/queryServicesMessage.o ./src/message/query/dynamicConfiguration/queryServicesMessage.c

./Build/queryJointForceTorquesMessage.o : ./src/message/query/manipulator/queryJointForceTorquesMessage.c
	gcc $(CCFLAGS) -o ./Build/queryJointForceTorquesMessage.o ./src/message/query/manipulator/queryJointForceTorquesMessage.c

./Build/queryJointVelocitiesMessage.o : ./src/message/query/manipulator/queryJointVelocitiesMessage.c
	gcc $(CCFLAGS) -o ./Build/queryJointVelocitiesMessage.o ./src/message/query/manipulator/queryJointVelocitiesMessage.c

./Build/queryJointPositionsMessage.o : ./src/message/query/manipulator/queryJointPositionsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryJointPositionsMessage.o ./src/message/query/manipulator/queryJointPositionsMessage.c

./Build/queryManipulatorSpecificationsMessage.o : ./src/message/query/manipulator/queryManipulatorSpecificationsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryManipulatorSpecificationsMessage.o ./src/message/query/manipulator/queryManipulatorSpecificationsMessage.c

./Build/queryJointEffortsMessage.o : ./src/message/query/manipulator/queryJointEffortsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryJointEffortsMessage.o ./src/message/query/manipulator/queryJointEffortsMessage.c

./Build/queryToolPointMessage.o : ./src/message/query/manipulator/queryToolPointMessage.c
	gcc $(CCFLAGS) -o ./Build/queryToolPointMessage.o ./src/message/query/manipulator/queryToolPointMessage.c

./Build/queryComponentAuthorityMessage.o : ./src/message/query/core/queryComponentAuthorityMessage.c
	gcc $(CCFLAGS) -o ./Build/queryComponentAuthorityMessage.o ./src/message/query/core/queryComponentAuthorityMessage.c

./Build/queryComponentStatusMessage.o : ./src/message/query/core/queryComponentStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/queryComponentStatusMessage.o ./src/message/query/core/queryComponentStatusMessage.c

./Build/queryComponentControlMessage.o : ./src/message/query/core/queryComponentControlMessage.c
	gcc $(CCFLAGS) -o ./Build/queryComponentControlMessage.o ./src/message/query/core/queryComponentControlMessage.c

./Build/queryTimeMessage.o : ./src/message/query/core/queryTimeMessage.c
	gcc $(CCFLAGS) -o ./Build/queryTimeMessage.o ./src/message/query/core/queryTimeMessage.c

./Build/queryLocalWaypointMessage.o : ./src/message/query/platform/queryLocalWaypointMessage.c
	gcc $(CCFLAGS) -o ./Build/queryLocalWaypointMessage.o ./src/message/query/platform/queryLocalWaypointMessage.c

./Build/queryDiscreteDevicesMessage.o : ./src/message/query/platform/queryDiscreteDevicesMessage.c
	gcc $(CCFLAGS) -o ./Build/queryDiscreteDevicesMessage.o ./src/message/query/platform/queryDiscreteDevicesMessage.c

./Build/queryPathSegmentCountMessage.o : ./src/message/query/platform/queryPathSegmentCountMessage.c
	gcc $(CCFLAGS) -o ./Build/queryPathSegmentCountMessage.o ./src/message/query/platform/queryPathSegmentCountMessage.c

./Build/queryPlatformOperationalDataMessage.o : ./src/message/query/platform/queryPlatformOperationalDataMessage.c
	gcc $(CCFLAGS) -o ./Build/queryPlatformOperationalDataMessage.o ./src/message/query/platform/queryPlatformOperationalDataMessage.c

./Build/queryPlatformSpecificationsMessage.o : ./src/message/query/platform/queryPlatformSpecificationsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryPlatformSpecificationsMessage.o ./src/message/query/platform/queryPlatformSpecificationsMessage.c

./Build/queryLocalVectorMessage.o : ./src/message/query/platform/queryLocalVectorMessage.c
	gcc $(CCFLAGS) -o ./Build/queryLocalVectorMessage.o ./src/message/query/platform/queryLocalVectorMessage.c

./Build/queryWaypointCountMessage.o : ./src/message/query/platform/queryWaypointCountMessage.c
	gcc $(CCFLAGS) -o ./Build/queryWaypointCountMessage.o ./src/message/query/platform/queryWaypointCountMessage.c

./Build/queryWrenchEffortMessage.o : ./src/message/query/platform/queryWrenchEffortMessage.c
	gcc $(CCFLAGS) -o ./Build/queryWrenchEffortMessage.o ./src/message/query/platform/queryWrenchEffortMessage.c

./Build/queryGlobalWaypointMessage.o : ./src/message/query/platform/queryGlobalWaypointMessage.c
	gcc $(CCFLAGS) -o ./Build/queryGlobalWaypointMessage.o ./src/message/query/platform/queryGlobalWaypointMessage.c

./Build/queryGlobalPathSegmentMessage.o : ./src/message/query/platform/queryGlobalPathSegmentMessage.c
	gcc $(CCFLAGS) -o ./Build/queryGlobalPathSegmentMessage.o ./src/message/query/platform/queryGlobalPathSegmentMessage.c

./Build/queryTravelSpeedMessage.o : ./src/message/query/platform/queryTravelSpeedMessage.c
	gcc $(CCFLAGS) -o ./Build/queryTravelSpeedMessage.o ./src/message/query/platform/queryTravelSpeedMessage.c

./Build/queryLocalPathSegmentMessage.o : ./src/message/query/platform/queryLocalPathSegmentMessage.c
	gcc $(CCFLAGS) -o ./Build/queryLocalPathSegmentMessage.o ./src/message/query/platform/queryLocalPathSegmentMessage.c

./Build/queryGlobalPoseMessage.o : ./src/message/query/platform/queryGlobalPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/queryGlobalPoseMessage.o ./src/message/query/platform/queryGlobalPoseMessage.c

./Build/queryLocalPoseMessage.o : ./src/message/query/platform/queryLocalPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/queryLocalPoseMessage.o ./src/message/query/platform/queryLocalPoseMessage.c

./Build/queryGlobalVectorMessage.o : ./src/message/query/platform/queryGlobalVectorMessage.c
	gcc $(CCFLAGS) -o ./Build/queryGlobalVectorMessage.o ./src/message/query/platform/queryGlobalVectorMessage.c

./Build/queryVelocityStateMessage.o : ./src/message/query/platform/queryVelocityStateMessage.c
	gcc $(CCFLAGS) -o ./Build/queryVelocityStateMessage.o ./src/message/query/platform/queryVelocityStateMessage.c

./Build/queryPayloadDataElementMessage.o : ./src/message/query/payload/queryPayloadDataElementMessage.c
	gcc $(CCFLAGS) -o ./Build/queryPayloadDataElementMessage.o ./src/message/query/payload/queryPayloadDataElementMessage.c

./Build/queryPayloadInterfaceMessage.o : ./src/message/query/payload/queryPayloadInterfaceMessage.c
	gcc $(CCFLAGS) -o ./Build/queryPayloadInterfaceMessage.o ./src/message/query/payload/queryPayloadInterfaceMessage.c

./Build/queryEventsMessage.o : ./src/message/query/event/queryEventsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryEventsMessage.o ./src/message/query/event/queryEventsMessage.c

./Build/querySpoolingPreferenceMessage.o : ./src/message/query/planning/querySpoolingPreferenceMessage.c
	gcc $(CCFLAGS) -o ./Build/querySpoolingPreferenceMessage.o ./src/message/query/planning/querySpoolingPreferenceMessage.c

./Build/queryMissionStatusMessage.o : ./src/message/query/planning/queryMissionStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/queryMissionStatusMessage.o ./src/message/query/planning/queryMissionStatusMessage.c

./Build/queryVksObjectsMessage.o : ./src/message/query/worldModel/queryVksObjectsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryVksObjectsMessage.o ./src/message/query/worldModel/queryVksObjectsMessage.c

./Build/queryVksBoundsMessage.o : ./src/message/query/worldModel/queryVksBoundsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryVksBoundsMessage.o ./src/message/query/worldModel/queryVksBoundsMessage.c

./Build/queryVksFeatureClassMetadataMessage.o : ./src/message/query/worldModel/queryVksFeatureClassMetadataMessage.c
	gcc $(CCFLAGS) -o ./Build/queryVksFeatureClassMetadataMessage.o ./src/message/query/worldModel/queryVksFeatureClassMetadataMessage.c

./Build/queryCameraFormatOptionsMessage.o : ./src/message/query/environment/queryCameraFormatOptionsMessage.c
	gcc $(CCFLAGS) -o ./Build/queryCameraFormatOptionsMessage.o ./src/message/query/environment/queryCameraFormatOptionsMessage.c

./Build/queryImageMessage.o : ./src/message/query/environment/queryImageMessage.c
	gcc $(CCFLAGS) -o ./Build/queryImageMessage.o ./src/message/query/environment/queryImageMessage.c

./Build/queryCameraPoseMessage.o : ./src/message/query/environment/queryCameraPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/queryCameraPoseMessage.o ./src/message/query/environment/queryCameraPoseMessage.c

./Build/queryCameraCapabilitiesMessage.o : ./src/message/query/environment/queryCameraCapabilitiesMessage.c
	gcc $(CCFLAGS) -o ./Build/queryCameraCapabilitiesMessage.o ./src/message/query/environment/queryCameraCapabilitiesMessage.c

./Build/queryCameraCountMessage.o : ./src/message/query/environment/queryCameraCountMessage.c
	gcc $(CCFLAGS) -o ./Build/queryCameraCountMessage.o ./src/message/query/environment/queryCameraCountMessage.c

./Build/queryRelativeObjectPositionMessage.o : ./src/message/query/environment/queryRelativeObjectPositionMessage.c
	gcc $(CCFLAGS) -o ./Build/queryRelativeObjectPositionMessage.o ./src/message/query/environment/queryRelativeObjectPositionMessage.c

./Build/querySelectedCameraMessage.o : ./src/message/query/environment/querySelectedCameraMessage.c
	gcc $(CCFLAGS) -o ./Build/querySelectedCameraMessage.o ./src/message/query/environment/querySelectedCameraMessage.c

./Build/reportSelectedDataLinkStatusMessage.o : ./src/message/inform/communications/reportSelectedDataLinkStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/reportSelectedDataLinkStatusMessage.o ./src/message/inform/communications/reportSelectedDataLinkStatusMessage.c

./Build/reportHeartbeatPulseMessage.o : ./src/message/inform/communications/reportHeartbeatPulseMessage.c
	gcc $(CCFLAGS) -o ./Build/reportHeartbeatPulseMessage.o ./src/message/inform/communications/reportHeartbeatPulseMessage.c

./Build/reportDataLinkStatusMessage.o : ./src/message/inform/communications/reportDataLinkStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/reportDataLinkStatusMessage.o ./src/message/inform/communications/reportDataLinkStatusMessage.c

./Build/reportServicesMessage.o : ./src/message/inform/dynamicConfiguration/reportServicesMessage.c
	gcc $(CCFLAGS) -o ./Build/reportServicesMessage.o ./src/message/inform/dynamicConfiguration/reportServicesMessage.c

./Build/reportConfigurationMessage.o : ./src/message/inform/dynamicConfiguration/reportConfigurationMessage.c
	gcc $(CCFLAGS) -o ./Build/reportConfigurationMessage.o ./src/message/inform/dynamicConfiguration/reportConfigurationMessage.c

./Build/reportSubsystemListMessage.o : ./src/message/inform/dynamicConfiguration/reportSubsystemListMessage.c
	gcc $(CCFLAGS) -o ./Build/reportSubsystemListMessage.o ./src/message/inform/dynamicConfiguration/reportSubsystemListMessage.c

./Build/reportIdentificationMessage.o : ./src/message/inform/dynamicConfiguration/reportIdentificationMessage.c
	gcc $(CCFLAGS) -o ./Build/reportIdentificationMessage.o ./src/message/inform/dynamicConfiguration/reportIdentificationMessage.c

./Build/reportToolPointMessage.o : ./src/message/inform/manipulator/reportToolPointMessage.c
	gcc $(CCFLAGS) -o ./Build/reportToolPointMessage.o ./src/message/inform/manipulator/reportToolPointMessage.c

./Build/reportJointEffortsMessage.o : ./src/message/inform/manipulator/reportJointEffortsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportJointEffortsMessage.o ./src/message/inform/manipulator/reportJointEffortsMessage.c

./Build/reportManipulatorSpecificationsMessage.o : ./src/message/inform/manipulator/reportManipulatorSpecificationsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportManipulatorSpecificationsMessage.o ./src/message/inform/manipulator/reportManipulatorSpecificationsMessage.c

./Build/reportJointVelocitiesMessage.o : ./src/message/inform/manipulator/reportJointVelocitiesMessage.c
	gcc $(CCFLAGS) -o ./Build/reportJointVelocitiesMessage.o ./src/message/inform/manipulator/reportJointVelocitiesMessage.c

./Build/reportJointForceTorquesMessage.o : ./src/message/inform/manipulator/reportJointForceTorquesMessage.c
	gcc $(CCFLAGS) -o ./Build/reportJointForceTorquesMessage.o ./src/message/inform/manipulator/reportJointForceTorquesMessage.c

./Build/reportJointPositionsMessage.o : ./src/message/inform/manipulator/reportJointPositionsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportJointPositionsMessage.o ./src/message/inform/manipulator/reportJointPositionsMessage.c

./Build/reportComponentControlMessage.o : ./src/message/inform/core/reportComponentControlMessage.c
	gcc $(CCFLAGS) -o ./Build/reportComponentControlMessage.o ./src/message/inform/core/reportComponentControlMessage.c

./Build/reportComponentStatusMessage.o : ./src/message/inform/core/reportComponentStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/reportComponentStatusMessage.o ./src/message/inform/core/reportComponentStatusMessage.c

./Build/reportTimeMessage.o : ./src/message/inform/core/reportTimeMessage.c
	gcc $(CCFLAGS) -o ./Build/reportTimeMessage.o ./src/message/inform/core/reportTimeMessage.c

./Build/reportComponentAuthorityMessage.o : ./src/message/inform/core/reportComponentAuthorityMessage.c
	gcc $(CCFLAGS) -o ./Build/reportComponentAuthorityMessage.o ./src/message/inform/core/reportComponentAuthorityMessage.c

./Build/reportLocalPoseMessage.o : ./src/message/inform/platform/reportLocalPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/reportLocalPoseMessage.o ./src/message/inform/platform/reportLocalPoseMessage.c

./Build/reportGlobalWaypointMessage.o : ./src/message/inform/platform/reportGlobalWaypointMessage.c
	gcc $(CCFLAGS) -o ./Build/reportGlobalWaypointMessage.o ./src/message/inform/platform/reportGlobalWaypointMessage.c

./Build/reportWrenchEffortMessage.o : ./src/message/inform/platform/reportWrenchEffortMessage.c
	gcc $(CCFLAGS) -o ./Build/reportWrenchEffortMessage.o ./src/message/inform/platform/reportWrenchEffortMessage.c

./Build/reportGlobalVectorMessage.o : ./src/message/inform/platform/reportGlobalVectorMessage.c
	gcc $(CCFLAGS) -o ./Build/reportGlobalVectorMessage.o ./src/message/inform/platform/reportGlobalVectorMessage.c

./Build/reportLocalWaypointMessage.o : ./src/message/inform/platform/reportLocalWaypointMessage.c
	gcc $(CCFLAGS) -o ./Build/reportLocalWaypointMessage.o ./src/message/inform/platform/reportLocalWaypointMessage.c

./Build/reportWaypointCountMessage.o : ./src/message/inform/platform/reportWaypointCountMessage.c
	gcc $(CCFLAGS) -o ./Build/reportWaypointCountMessage.o ./src/message/inform/platform/reportWaypointCountMessage.c

./Build/reportDiscreteDevicesMessage.o : ./src/message/inform/platform/reportDiscreteDevicesMessage.c
	gcc $(CCFLAGS) -o ./Build/reportDiscreteDevicesMessage.o ./src/message/inform/platform/reportDiscreteDevicesMessage.c

./Build/reportPlatformSpecificationsMessage.o : ./src/message/inform/platform/reportPlatformSpecificationsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportPlatformSpecificationsMessage.o ./src/message/inform/platform/reportPlatformSpecificationsMessage.c

./Build/reportPlatformOperationalDataMessage.o : ./src/message/inform/platform/reportPlatformOperationalDataMessage.c
	gcc $(CCFLAGS) -o ./Build/reportPlatformOperationalDataMessage.o ./src/message/inform/platform/reportPlatformOperationalDataMessage.c

./Build/reportGlobalPoseMessage.o : ./src/message/inform/platform/reportGlobalPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/reportGlobalPoseMessage.o ./src/message/inform/platform/reportGlobalPoseMessage.c

./Build/reportGlobalPathSegmentMessage.o : ./src/message/inform/platform/reportGlobalPathSegmentMessage.c
	gcc $(CCFLAGS) -o ./Build/reportGlobalPathSegmentMessage.o ./src/message/inform/platform/reportGlobalPathSegmentMessage.c

./Build/reportTravelSpeedMessage.o : ./src/message/inform/platform/reportTravelSpeedMessage.c
	gcc $(CCFLAGS) -o ./Build/reportTravelSpeedMessage.o ./src/message/inform/platform/reportTravelSpeedMessage.c

./Build/reportLocalPathSegmentMessage.o : ./src/message/inform/platform/reportLocalPathSegmentMessage.c
	gcc $(CCFLAGS) -o ./Build/reportLocalPathSegmentMessage.o ./src/message/inform/platform/reportLocalPathSegmentMessage.c

./Build/reportLocalVectorMessage.o : ./src/message/inform/platform/reportLocalVectorMessage.c
	gcc $(CCFLAGS) -o ./Build/reportLocalVectorMessage.o ./src/message/inform/platform/reportLocalVectorMessage.c

./Build/reportVelocityStateMessage.o : ./src/message/inform/platform/reportVelocityStateMessage.c
	gcc $(CCFLAGS) -o ./Build/reportVelocityStateMessage.o ./src/message/inform/platform/reportVelocityStateMessage.c

./Build/reportPathSegmentCountMessage.o : ./src/message/inform/platform/reportPathSegmentCountMessage.c
	gcc $(CCFLAGS) -o ./Build/reportPathSegmentCountMessage.o ./src/message/inform/platform/reportPathSegmentCountMessage.c

./Build/reportPayloadDataElementMessage.o : ./src/message/inform/payload/reportPayloadDataElementMessage.c
	gcc $(CCFLAGS) -o ./Build/reportPayloadDataElementMessage.o ./src/message/inform/payload/reportPayloadDataElementMessage.c

./Build/reportPayloadInterfaceMessage.o : ./src/message/inform/payload/reportPayloadInterfaceMessage.c
	gcc $(CCFLAGS) -o ./Build/reportPayloadInterfaceMessage.o ./src/message/inform/payload/reportPayloadInterfaceMessage.c

./Build/eventMessage.o : ./src/message/inform/event/eventMessage.c
	gcc $(CCFLAGS) -o ./Build/eventMessage.o ./src/message/inform/event/eventMessage.c

./Build/reportEventsMessage.o : ./src/message/inform/event/reportEventsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportEventsMessage.o ./src/message/inform/event/reportEventsMessage.c

./Build/reportMissionStatusMessage.o : ./src/message/inform/planning/reportMissionStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/reportMissionStatusMessage.o ./src/message/inform/planning/reportMissionStatusMessage.c

./Build/reportSpoolingPreferenceMessage.o : ./src/message/inform/planning/reportSpoolingPreferenceMessage.c
	gcc $(CCFLAGS) -o ./Build/reportSpoolingPreferenceMessage.o ./src/message/inform/planning/reportSpoolingPreferenceMessage.c

./Build/reportVksDataTransferTerminationMessage.o : ./src/message/inform/worldModel/reportVksDataTransferTerminationMessage.c
	gcc $(CCFLAGS) -o ./Build/reportVksDataTransferTerminationMessage.o ./src/message/inform/worldModel/reportVksDataTransferTerminationMessage.c

./Build/reportVksFeatureClassMetadataMessage.o : ./src/message/inform/worldModel/reportVksFeatureClassMetadataMessage.c
	gcc $(CCFLAGS) -o ./Build/reportVksFeatureClassMetadataMessage.o ./src/message/inform/worldModel/reportVksFeatureClassMetadataMessage.c

./Build/reportVksObjectsCreationMessage.o : ./src/message/inform/worldModel/reportVksObjectsCreationMessage.c
	gcc $(CCFLAGS) -o ./Build/reportVksObjectsCreationMessage.o ./src/message/inform/worldModel/reportVksObjectsCreationMessage.c

./Build/reportVksBoundsMessage.o : ./src/message/inform/worldModel/reportVksBoundsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportVksBoundsMessage.o ./src/message/inform/worldModel/reportVksBoundsMessage.c

./Build/reportVksObjectsMessage.o : ./src/message/inform/worldModel/reportVksObjectsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportVksObjectsMessage.o ./src/message/inform/worldModel/reportVksObjectsMessage.c

./Build/reportImageMessage.o : ./src/message/inform/environment/reportImageMessage.c
	gcc $(CCFLAGS) -o ./Build/reportImageMessage.o ./src/message/inform/environment/reportImageMessage.c

./Build/reportCameraFormatOptionsMessage.o : ./src/message/inform/environment/reportCameraFormatOptionsMessage.c
	gcc $(CCFLAGS) -o ./Build/reportCameraFormatOptionsMessage.o ./src/message/inform/environment/reportCameraFormatOptionsMessage.c

./Build/reportSelectedCameraMessage.o : ./src/message/inform/environment/reportSelectedCameraMessage.c
	gcc $(CCFLAGS) -o ./Build/reportSelectedCameraMessage.o ./src/message/inform/environment/reportSelectedCameraMessage.c

./Build/reportCameraCapabilitiesMessage.o : ./src/message/inform/environment/reportCameraCapabilitiesMessage.c
	gcc $(CCFLAGS) -o ./Build/reportCameraCapabilitiesMessage.o ./src/message/inform/environment/reportCameraCapabilitiesMessage.c

./Build/reportCameraCountMessage.o : ./src/message/inform/environment/reportCameraCountMessage.c
	gcc $(CCFLAGS) -o ./Build/reportCameraCountMessage.o ./src/message/inform/environment/reportCameraCountMessage.c

./Build/reportRelativeObjectPositionMessage.o : ./src/message/inform/environment/reportRelativeObjectPositionMessage.c
	gcc $(CCFLAGS) -o ./Build/reportRelativeObjectPositionMessage.o ./src/message/inform/environment/reportRelativeObjectPositionMessage.c

./Build/reportCameraPoseMessage.o : ./src/message/inform/environment/reportCameraPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/reportCameraPoseMessage.o ./src/message/inform/environment/reportCameraPoseMessage.c

./Build/jausMessage.o : ./src/message/jausMessage.c
	gcc $(CCFLAGS) -o ./Build/jausMessage.o ./src/message/jausMessage.c

./Build/setDataLinkStatusMessage.o : ./src/message/command/communications/setDataLinkStatusMessage.c
	gcc $(CCFLAGS) -o ./Build/setDataLinkStatusMessage.o ./src/message/command/communications/setDataLinkStatusMessage.c

./Build/setSelectedDataLinkStateMessage.o : ./src/message/command/communications/setSelectedDataLinkStateMessage.c
	gcc $(CCFLAGS) -o ./Build/setSelectedDataLinkStateMessage.o ./src/message/command/communications/setSelectedDataLinkStateMessage.c

./Build/setDataLinkSelectMessage.o : ./src/message/command/communications/setDataLinkSelectMessage.c
	gcc $(CCFLAGS) -o ./Build/setDataLinkSelectMessage.o ./src/message/command/communications/setDataLinkSelectMessage.c

./Build/setJointPositionsMessage.o : ./src/message/command/manipulator/setJointPositionsMessage.c
	gcc $(CCFLAGS) -o ./Build/setJointPositionsMessage.o ./src/message/command/manipulator/setJointPositionsMessage.c

./Build/setEndEffectorPoseMessage.o : ./src/message/command/manipulator/setEndEffectorPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/setEndEffectorPoseMessage.o ./src/message/command/manipulator/setEndEffectorPoseMessage.c

./Build/setJointMotionMessage.o : ./src/message/command/manipulator/setJointMotionMessage.c
	gcc $(CCFLAGS) -o ./Build/setJointMotionMessage.o ./src/message/command/manipulator/setJointMotionMessage.c

./Build/setToolPointMessage.o : ./src/message/command/manipulator/setToolPointMessage.c
	gcc $(CCFLAGS) -o ./Build/setToolPointMessage.o ./src/message/command/manipulator/setToolPointMessage.c

./Build/setEndEffectorVelocityStateMessage.o : ./src/message/command/manipulator/setEndEffectorVelocityStateMessage.c
	gcc $(CCFLAGS) -o ./Build/setEndEffectorVelocityStateMessage.o ./src/message/command/manipulator/setEndEffectorVelocityStateMessage.c

./Build/setJointVelocitiesMessage.o : ./src/message/command/manipulator/setJointVelocitiesMessage.c
	gcc $(CCFLAGS) -o ./Build/setJointVelocitiesMessage.o ./src/message/command/manipulator/setJointVelocitiesMessage.c

./Build/setEndEffectorPathMotionMessage.o : ./src/message/command/manipulator/setEndEffectorPathMotionMessage.c
	gcc $(CCFLAGS) -o ./Build/setEndEffectorPathMotionMessage.o ./src/message/command/manipulator/setEndEffectorPathMotionMessage.c

./Build/setJointEffortsMessage.o : ./src/message/command/manipulator/setJointEffortsMessage.c
	gcc $(CCFLAGS) -o ./Build/setJointEffortsMessage.o ./src/message/command/manipulator/setJointEffortsMessage.c

./Build/setEmergencyMessage.o : ./src/message/command/core/setEmergencyMessage.c
	gcc $(CCFLAGS) -o ./Build/setEmergencyMessage.o ./src/message/command/core/setEmergencyMessage.c

./Build/standbyMessage.o : ./src/message/command/core/standbyMessage.c
	gcc $(CCFLAGS) -o ./Build/standbyMessage.o ./src/message/command/core/standbyMessage.c

./Build/confirmServiceConnectionMessage.o : ./src/message/command/core/confirmServiceConnectionMessage.c
	gcc $(CCFLAGS) -o ./Build/confirmServiceConnectionMessage.o ./src/message/command/core/confirmServiceConnectionMessage.c

./Build/releaseComponentControlMessage.o : ./src/message/command/core/releaseComponentControlMessage.c
	gcc $(CCFLAGS) -o ./Build/releaseComponentControlMessage.o ./src/message/command/core/releaseComponentControlMessage.c

./Build/suspendServiceConnectionMessage.o : ./src/message/command/core/suspendServiceConnectionMessage.c
	gcc $(CCFLAGS) -o ./Build/suspendServiceConnectionMessage.o ./src/message/command/core/suspendServiceConnectionMessage.c

./Build/resumeMessage.o : ./src/message/command/core/resumeMessage.c
	gcc $(CCFLAGS) -o ./Build/resumeMessage.o ./src/message/command/core/resumeMessage.c

./Build/setTimeMessage.o : ./src/message/command/core/setTimeMessage.c
	gcc $(CCFLAGS) -o ./Build/setTimeMessage.o ./src/message/command/core/setTimeMessage.c

./Build/confirmComponentControlMessage.o : ./src/message/command/core/confirmComponentControlMessage.c
	gcc $(CCFLAGS) -o ./Build/confirmComponentControlMessage.o ./src/message/command/core/confirmComponentControlMessage.c

./Build/shutdownMessage.o : ./src/message/command/core/shutdownMessage.c
	gcc $(CCFLAGS) -o ./Build/shutdownMessage.o ./src/message/command/core/shutdownMessage.c

./Build/requestComponentControlMessage.o : ./src/message/command/core/requestComponentControlMessage.c
	gcc $(CCFLAGS) -o ./Build/requestComponentControlMessage.o ./src/message/command/core/requestComponentControlMessage.c

./Build/setComponentAuthorityMessage.o : ./src/message/command/core/setComponentAuthorityMessage.c
	gcc $(CCFLAGS) -o ./Build/setComponentAuthorityMessage.o ./src/message/command/core/setComponentAuthorityMessage.c

./Build/resetMessage.o : ./src/message/command/core/resetMessage.c
	gcc $(CCFLAGS) -o ./Build/resetMessage.o ./src/message/command/core/resetMessage.c

./Build/rejectComponentControlMessage.o : ./src/message/command/core/rejectComponentControlMessage.c
	gcc $(CCFLAGS) -o ./Build/rejectComponentControlMessage.o ./src/message/command/core/rejectComponentControlMessage.c

./Build/terminateServiceConnectionMessage.o : ./src/message/command/core/terminateServiceConnectionMessage.c
	gcc $(CCFLAGS) -o ./Build/terminateServiceConnectionMessage.o ./src/message/command/core/terminateServiceConnectionMessage.c

./Build/activateServiceConnectionMessage.o : ./src/message/command/core/activateServiceConnectionMessage.c
	gcc $(CCFLAGS) -o ./Build/activateServiceConnectionMessage.o ./src/message/command/core/activateServiceConnectionMessage.c

./Build/clearEmergencyMessage.o : ./src/message/command/core/clearEmergencyMessage.c
	gcc $(CCFLAGS) -o ./Build/clearEmergencyMessage.o ./src/message/command/core/clearEmergencyMessage.c

./Build/createServiceConnectionMessage.o : ./src/message/command/core/createServiceConnectionMessage.c
	gcc $(CCFLAGS) -o ./Build/createServiceConnectionMessage.o ./src/message/command/core/createServiceConnectionMessage.c

./Build/setTravelSpeedMessage.o : ./src/message/command/platform/setTravelSpeedMessage.c
	gcc $(CCFLAGS) -o ./Build/setTravelSpeedMessage.o ./src/message/command/platform/setTravelSpeedMessage.c

./Build/setGlobalPathSegmentMessage.o : ./src/message/command/platform/setGlobalPathSegmentMessage.c
	gcc $(CCFLAGS) -o ./Build/setGlobalPathSegmentMessage.o ./src/message/command/platform/setGlobalPathSegmentMessage.c

./Build/setWrenchEffortMessage.o : ./src/message/command/platform/setWrenchEffortMessage.c
	gcc $(CCFLAGS) -o ./Build/setWrenchEffortMessage.o ./src/message/command/platform/setWrenchEffortMessage.c

./Build/setGlobalVectorMessage.o : ./src/message/command/platform/setGlobalVectorMessage.c
	gcc $(CCFLAGS) -o ./Build/setGlobalVectorMessage.o ./src/message/command/platform/setGlobalVectorMessage.c

./Build/setDiscreteDevicesMessage.o : ./src/message/command/platform/setDiscreteDevicesMessage.c
	gcc $(CCFLAGS) -o ./Build/setDiscreteDevicesMessage.o ./src/message/command/platform/setDiscreteDevicesMessage.c

./Build/setLocalVectorMessage.o : ./src/message/command/platform/setLocalVectorMessage.c
	gcc $(CCFLAGS) -o ./Build/setLocalVectorMessage.o ./src/message/command/platform/setLocalVectorMessage.c

./Build/setVelocityStateMessage.o : ./src/message/command/platform/setVelocityStateMessage.c
	gcc $(CCFLAGS) -o ./Build/setVelocityStateMessage.o ./src/message/command/platform/setVelocityStateMessage.c

./Build/setLocalPathSegmentMessage.o : ./src/message/command/platform/setLocalPathSegmentMessage.c
	gcc $(CCFLAGS) -o ./Build/setLocalPathSegmentMessage.o ./src/message/command/platform/setLocalPathSegmentMessage.c

./Build/setLocalWaypointMessage.o : ./src/message/command/platform/setLocalWaypointMessage.c
	gcc $(CCFLAGS) -o ./Build/setLocalWaypointMessage.o ./src/message/command/platform/setLocalWaypointMessage.c

./Build/setGlobalWaypointMessage.o : ./src/message/command/platform/setGlobalWaypointMessage.c
	gcc $(CCFLAGS) -o ./Build/setGlobalWaypointMessage.o ./src/message/command/platform/setGlobalWaypointMessage.c

./Build/setPayloadDataElementMessage.o : ./src/message/command/payload/setPayloadDataElementMessage.c
	gcc $(CCFLAGS) -o ./Build/setPayloadDataElementMessage.o ./src/message/command/payload/setPayloadDataElementMessage.c

./Build/createEventMessage.o : ./src/message/command/event/createEventMessage.c
	gcc $(CCFLAGS) -o ./Build/createEventMessage.o ./src/message/command/event/createEventMessage.c

./Build/confirmEventRequestMessage.o : ./src/message/command/event/confirmEventRequestMessage.c
	gcc $(CCFLAGS) -o ./Build/confirmEventRequestMessage.o ./src/message/command/event/confirmEventRequestMessage.c

./Build/cancelEventMessage.o : ./src/message/command/event/cancelEventMessage.c
	gcc $(CCFLAGS) -o ./Build/cancelEventMessage.o ./src/message/command/event/cancelEventMessage.c

./Build/updateEventMessage.o : ./src/message/command/event/updateEventMessage.c
	gcc $(CCFLAGS) -o ./Build/updateEventMessage.o ./src/message/command/event/updateEventMessage.c

./Build/rejectEventRequestMessage.o : ./src/message/command/event/rejectEventRequestMessage.c
	gcc $(CCFLAGS) -o ./Build/rejectEventRequestMessage.o ./src/message/command/event/rejectEventRequestMessage.c

./Build/pauseMissionMessage.o : ./src/message/command/planning/pauseMissionMessage.c
	gcc $(CCFLAGS) -o ./Build/pauseMissionMessage.o ./src/message/command/planning/pauseMissionMessage.c

./Build/spoolMissionMessage.o : ./src/message/command/planning/spoolMissionMessage.c
	gcc $(CCFLAGS) -o ./Build/spoolMissionMessage.o ./src/message/command/planning/spoolMissionMessage.c

./Build/removeMessagesMessage.o : ./src/message/command/planning/removeMessagesMessage.c
	gcc $(CCFLAGS) -o ./Build/removeMessagesMessage.o ./src/message/command/planning/removeMessagesMessage.c

./Build/runMissionMessage.o : ./src/message/command/planning/runMissionMessage.c
	gcc $(CCFLAGS) -o ./Build/runMissionMessage.o ./src/message/command/planning/runMissionMessage.c

./Build/resumeMissionMessage.o : ./src/message/command/planning/resumeMissionMessage.c
	gcc $(CCFLAGS) -o ./Build/resumeMissionMessage.o ./src/message/command/planning/resumeMissionMessage.c

./Build/replaceMessagesMessage.o : ./src/message/command/planning/replaceMessagesMessage.c
	gcc $(CCFLAGS) -o ./Build/replaceMessagesMessage.o ./src/message/command/planning/replaceMessagesMessage.c

./Build/abortMissionMessage.o : ./src/message/command/planning/abortMissionMessage.c
	gcc $(CCFLAGS) -o ./Build/abortMissionMessage.o ./src/message/command/planning/abortMissionMessage.c

./Build/terminateVksDataTransferMessage.o : ./src/message/command/worldModel/terminateVksDataTransferMessage.c
	gcc $(CCFLAGS) -o ./Build/terminateVksDataTransferMessage.o ./src/message/command/worldModel/terminateVksDataTransferMessage.c

./Build/setVksFeatureClassMetadataMessage.o : ./src/message/command/worldModel/setVksFeatureClassMetadataMessage.c
	gcc $(CCFLAGS) -o ./Build/setVksFeatureClassMetadataMessage.o ./src/message/command/worldModel/setVksFeatureClassMetadataMessage.c

./Build/createVksObjectsMessage.o : ./src/message/command/worldModel/createVksObjectsMessage.c
	gcc $(CCFLAGS) -o ./Build/createVksObjectsMessage.o ./src/message/command/worldModel/createVksObjectsMessage.c

./Build/deleteVksObjectsMessage.o : ./src/message/command/worldModel/deleteVksObjectsMessage.c
	gcc $(CCFLAGS) -o ./Build/deleteVksObjectsMessage.o ./src/message/command/worldModel/deleteVksObjectsMessage.c

./Build/setCameraFormatOptionsMessage.o : ./src/message/command/environment/setCameraFormatOptionsMessage.c
	gcc $(CCFLAGS) -o ./Build/setCameraFormatOptionsMessage.o ./src/message/command/environment/setCameraFormatOptionsMessage.c

./Build/setCameraCapabilitiesMessage.o : ./src/message/command/environment/setCameraCapabilitiesMessage.c
	gcc $(CCFLAGS) -o ./Build/setCameraCapabilitiesMessage.o ./src/message/command/environment/setCameraCapabilitiesMessage.c

./Build/setCameraPoseMessage.o : ./src/message/command/environment/setCameraPoseMessage.c
	gcc $(CCFLAGS) -o ./Build/setCameraPoseMessage.o ./src/message/command/environment/setCameraPoseMessage.c

./Build/selectCameraMessage.o : ./src/message/command/environment/selectCameraMessage.c
	gcc $(CCFLAGS) -o ./Build/selectCameraMessage.o ./src/message/command/environment/selectCameraMessage.c

./Build/jausSubsystem.o : ./src/jausSubsystem.c
	gcc $(CCFLAGS) -o ./Build/jausSubsystem.o ./src/jausSubsystem.c
