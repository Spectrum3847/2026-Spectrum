//! Generated from `ctre/phoenix/StatusCodes.h` and the Java `StatusCode` descriptions (26.3.0).
#![allow(non_upper_case_globals)]

use std::fmt;

/// A Phoenix status code. Zero is OK, positive values are warnings, negative values are errors.
///
/// Java's `StatusCode.valueOf` turns an unknown number into `OK`. This type keeps the number.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct StatusCode(pub i32);

#[rustfmt::skip]
impl StatusCode {
    pub const OK: Self = Self(0);
    pub const TaskIsBusy: Self = Self(-100);
    pub const InvalidDeviceSpec: Self = Self(-101);
    pub const EcuIsNotPresent: Self = Self(-102);
    pub const CouldNotEnterBl: Self = Self(-103);
    pub const CouldNotConfirmBl: Self = Self(-104);
    pub const CouldNotErase: Self = Self(-105);
    pub const CouldNotSendFlash: Self = Self(-106);
    pub const CouldNotValidate: Self = Self(-107);
    pub const CouldNotRunApp: Self = Self(-108);
    pub const CouldNotReqSetId: Self = Self(-109);
    pub const CouldNotConfirmId: Self = Self(-110);
    pub const FlashWasGood: Self = Self(-111);
    pub const AppTooOld: Self = Self(-112);
    pub const CouldNotReqSetDesc: Self = Self(-113);
    pub const CompileSzIsWrong: Self = Self(-114);
    pub const GadgeteerDeviceNoSetId: Self = Self(-115);
    pub const InvalidTask: Self = Self(-116);
    pub const NotImplemented: Self = Self(-117);
    pub const NoDevicesOnBus: Self = Self(-118);
    pub const MoreThanOneFile: Self = Self(-119);
    pub const NodeIsInvalid: Self = Self(-120);
    pub const InvalidDeviceDescriptor: Self = Self(-121);
    pub const CouldNotSendCanFrame: Self = Self(-123);
    pub const NormalModeMsgNotPresent: Self = Self(-124);
    pub const FeatureNotSupported: Self = Self(-125);
    pub const NotUpdating: Self = Self(-126);
    pub const CorruptedPOST: Self = Self(-127);
    pub const NoConfigs: Self = Self(-128);
    pub const ConfigFailed: Self = Self(-129);
    pub const CouldNotReqFactoryDefault: Self = Self(-130);
    pub const CustomNameNotSupported: Self = Self(-131);
    pub const ConfigReadWriteMismatch: Self = Self(-132);
    pub const CouldNotReqSetConfigs: Self = Self(-133);
    pub const InsufficientSz: Self = Self(-134);
    pub const InvalidModel: Self = Self(-135);
    pub const CouldNotReqDevInfo: Self = Self(-140);
    pub const NoControls: Self = Self(-141);
    pub const DeviceIsNull: Self = Self(-142);
    pub const DeviceDidNotRespondToDiagReq: Self = Self(-143);
    pub const OnlySupportedInTunerX: Self = Self(-144);
    pub const CanivCliError: Self = Self(-145);
    pub const InvalidCrfBadHeader: Self = Self(-200);
    pub const InvalidCrfFileSzInvald: Self = Self(-201);
    pub const InvalidCrfWrongProduct: Self = Self(-202);
    pub const InvalidCrfNoSects: Self = Self(-203);
    pub const InvalidCrfBadSectHeader: Self = Self(-204);
    pub const InvalidCrfBadSectSize: Self = Self(-205);
    pub const NoCrfFile: Self = Self(-206);
    pub const CouldNotFindDynamicId: Self = Self(-300);
    pub const DidNotGetDhcp: Self = Self(-301);
    pub const DidNotGetFullDhcp: Self = Self(-302);
    pub const InvalidLicenseResp: Self = Self(-350);
    pub const InvalidCanivCache: Self = Self(-351);
    pub const CannotOpenSerialPort: Self = Self(-500);
    pub const CannotWriteSerialPort: Self = Self(-501);
    pub const CannotReadSerialPort: Self = Self(-502);
    pub const CannotSerialToDevice: Self = Self(-503);
    pub const NoSerialControlFrameResp: Self = Self(-504);
    pub const CannotOpenUdpPort: Self = Self(-600);
    pub const CannotWriteUdpPort: Self = Self(-601);
    pub const CannotReadUdpPort: Self = Self(-602);
    pub const CannotUdpToDevice: Self = Self(-603);
    pub const NoUdpControlFrameResp: Self = Self(-604);
    pub const TimeoutIso15Response: Self = Self(-605);
    pub const InvalidJson: Self = Self(-700);
    pub const AppIsTerminating: Self = Self(-800);
    pub const CanMessageStale: Self = Self(1000);
    pub const BufferFull: Self = Self(1006);
    pub const PulseWidthSensorNotPresent: Self = Self(1010);
    pub const GeneralWarning: Self = Self(1100);
    pub const FirmVersionCouldNotBeRetrieved: Self = Self(1103);
    pub const FeaturesNotAvailableYet: Self = Self(1104);
    pub const ControlModeNotValid: Self = Self(1105);
    pub const ControlModeNotSupportedYet: Self = Self(1106);
    pub const MotProfFirmThreshold: Self = Self(1109);
    pub const MotProfFirmThreshold2: Self = Self(1110);
    pub const SimDeviceNotFound: Self = Self(1200);
    pub const SimPhysicsTypeNotSupported: Self = Self(1201);
    pub const SimDeviceAlreadyExists: Self = Self(1202);
    pub const TxFailed: Self = Self(-1001);
    pub const InvalidParamValue: Self = Self(-1002);
    pub const RxTimeout: Self = Self(-1003);
    pub const TxTimeout: Self = Self(-1004);
    pub const UnexpectedArbId: Self = Self(-1005);
    pub const CanOverflowed: Self = Self(-1006);
    pub const SensorNotPresent: Self = Self(-1007);
    pub const FirmwareTooOld: Self = Self(-1008);
    pub const CouldNotChangePeriod: Self = Self(-1009);
    pub const BufferFailure: Self = Self(-1010);
    pub const FirmwareNonFRC: Self = Self(-1011);
    pub const GeneralError: Self = Self(-1100);
    pub const SigNotUpdated: Self = Self(-1200);
    pub const NotAllPIDValuesUpdated: Self = Self(-1201);
    pub const GEN_PORT_ERROR: Self = Self(-1300);
    pub const PORT_MODULE_TYPE_MISMATCH: Self = Self(-1301);
    pub const GEN_MODULE_ERROR: Self = Self(-1400);
    pub const MODULE_NOT_INIT_SET_ERROR: Self = Self(-1401);
    pub const MODULE_NOT_INIT_GET_ERROR: Self = Self(-1402);
    pub const WheelRadiusTooSmall: Self = Self(-1500);
    pub const TicksPerRevZero: Self = Self(-1501);
    pub const DistanceBetweenWheelsTooSmall: Self = Self(-1502);
    pub const GainsAreNotSet: Self = Self(-1503);
    pub const WrongRemoteLimitSwitchSource: Self = Self(-1504);
    pub const DoubleVoltageCompensatingWPI: Self = Self(-1505);
    pub const CANdleAnimSlotOutOfBounds: Self = Self(-1506);
    pub const IncompatibleMode: Self = Self(-1600);
    pub const InvalidHandle: Self = Self(-1601);
    pub const FeatureRequiresHigherFirm: Self = Self(-1700);
    pub const ConfigFactoryDefaultRequiresHigherFirm: Self = Self(-1702);
    pub const ConfigMotionSCurveRequiresHigherFirm: Self = Self(-1703);
    pub const TalonFXFirmwarePreVBatDetect: Self = Self(-1704);
    pub const CANdleAnimationsRequireHigherFirm: Self = Self(-1705);
    pub const LibraryCouldNotBeLoaded: Self = Self(-1800);
    pub const MissingRoutineInLibrary: Self = Self(-1801);
    pub const ResourceNotAvailable: Self = Self(-1802);
    pub const MusicFileNotFound: Self = Self(-1900);
    pub const MusicFileWrongSize: Self = Self(-1901);
    pub const MusicFileTooNew: Self = Self(-1902);
    pub const MusicFileInvalid: Self = Self(-1903);
    pub const InvalidOrchestraAction: Self = Self(-1904);
    pub const MusicFileTooOld: Self = Self(-1905);
    pub const MusicInterrupted: Self = Self(-1906);
    pub const MusicNotSupported: Self = Self(-1907);
    pub const kInvalidInterface: Self = Self(-2000);
    pub const kInvalidGuid: Self = Self(-2001);
    pub const kInvalidClass: Self = Self(-2002);
    pub const kInvalidProtocol: Self = Self(-2003);
    pub const kInvalidPath: Self = Self(-2004);
    pub const kGeneralWinUsbError: Self = Self(-2005);
    pub const kFailedSetup: Self = Self(-2006);
    pub const kListenFailed: Self = Self(-2007);
    pub const kSendFailed: Self = Self(-2008);
    pub const kReceiveFailed: Self = Self(-2009);
    pub const kInvalidRespFormat: Self = Self(-2010);
    pub const kWinUsbInitFailed: Self = Self(-2011);
    pub const kWinUsbQueryFailed: Self = Self(-2012);
    pub const kWinUsbGeneralError: Self = Self(-2013);
    pub const kAccessDenied: Self = Self(-2014);
    pub const kFirmwareInvalidResponse: Self = Self(-2015);
    pub const StatusCodeNotInitialized: Self = Self(-10000);
    pub const WarningNotInitialized: Self = Self(10000);
    pub const HwTimestampOutOfSync: Self = Self(10001);
    pub const FrequentConfigCalls: Self = Self(10002);
    pub const InvalidNetwork: Self = Self(-10001);
    pub const MultiSignalNotSupported: Self = Self(-10002);
    pub const CouldNotCast: Self = Self(-10003);
    pub const NotFound: Self = Self(-10004);
    pub const NotSupported: Self = Self(-10005);
    pub const MissingContext: Self = Self(-10006);
    pub const ModelMismatch: Self = Self(-10007);
    pub const SerialMismatch: Self = Self(-10008);
    pub const NoFile: Self = Self(-10009);
    pub const LicenseDownloadFailed: Self = Self(-10010);
    pub const SelfTestIsEmpty: Self = Self(-10011);
    pub const SignalLookupFailed: Self = Self(-10012);
    pub const InvalidModeToGetSignal: Self = Self(-10013);
    pub const UnlicensedDevice: Self = Self(-10014);
    pub const InvalidSize: Self = Self(-10015);
    pub const InvalidLicenseResponse: Self = Self(-10016);
    pub const InvalidContext: Self = Self(-10017);
    pub const InternalError: Self = Self(-10018);
    pub const kDeviceResponseIncorrect: Self = Self(-10019);
    pub const kErrorPollingForDevices: Self = Self(-10020);
    pub const CouldNotRetrieveV6Firmware: Self = Self(-10021);
    pub const CouldNotDecodeDeviceFirmware: Self = Self(-10022);
    pub const InvalidIDToFollow: Self = Self(-10023);
    pub const UsingProFeatureOnUnlicensedDevice: Self = Self(-10024);
    pub const FirmwareTooNew: Self = Self(-10025);
    pub const CouldNotSerialize: Self = Self(-10026);
    pub const MechanismFaulted: Self = Self(-10027);
    pub const FirmwareVersNotCompatible: Self = Self(-10028);
    pub const DirectoryMissing: Self = Self(-10029);
    pub const ApiTooOld: Self = Self(-10030);
    pub const LoggerNotRunning: Self = Self(-10031);
    pub const TimeoutCannotBeZero: Self = Self(-10032);
    pub const CannotLicenseWhileEnabled: Self = Self(-10033);
    pub const InvalidFile: Self = Self(-10034);
    pub const HootLogTooOld: Self = Self(-10035);
    pub const HootLogTooNew: Self = Self(-10036);
    pub const UnlicensedHootLog: Self = Self(-10037);
    pub const CannotStepWhileUnpaused: Self = Self(-10038);
    pub const MultipleReplayNotSupported: Self = Self(-10039);
    pub const UserSignalLimitExceeded: Self = Self(-10040);
    pub const InvalidDeviceModel: Self = Self(-10041);
    pub const SignalNotSupportedOrMissing: Self = Self(-10042);
    pub const MissingSchema: Self = Self(-10043);
    pub const InvalidDeviceId: Self = Self(-10044);
}

/// Value, name, description.
#[rustfmt::skip]
static TABLE: &[(i32, &str, &str)] = &[
    (0, "OK", "No Error"),
    (-100, "TaskIsBusy", "Diagnostic Server is busy with another command."),
    (-101, "InvalidDeviceSpec", "InvalidDeviceSpec"),
    (-102, "EcuIsNotPresent", "Device is not present. Verify the device is connected and powered, and that the CAN bus is terminated."),
    (-103, "CouldNotEnterBl", "Could not put the device into bootloader mode."),
    (-104, "CouldNotConfirmBl", "Could not confirm the device has entered the bootloader."),
    (-105, "CouldNotErase", "Could not erase flash."),
    (-106, "CouldNotSendFlash", "Could not field upgrade the device."),
    (-107, "CouldNotValidate", "Bootloader could not verify integrity of the flashed application."),
    (-108, "CouldNotRunApp", "Could not run the device firmware application."),
    (-109, "CouldNotReqSetId", "Unable to set ID to this device. Confirm that this action is valid for this device and the ID is valid (0-62)."),
    (-110, "CouldNotConfirmId", "Could not verify that the changed ID took effect."),
    (-111, "FlashWasGood", "Device field upgrade was successful."),
    (-112, "AppTooOld", "Device firmware application is too old."),
    (-113, "CouldNotReqSetDesc", "Unable to set name to this device."),
    (-114, "CompileSzIsWrong", "CompileSzIsWrong"),
    (-115, "GadgeteerDeviceNoSetId", "Cannot set the ID of a gadgeteer device."),
    (-116, "InvalidTask", "This diagnostic action is not supported."),
    (-117, "NotImplemented", "Not Implemented, check latest installer."),
    (-118, "NoDevicesOnBus", "NoDevicesOnBus"),
    (-119, "MoreThanOneFile", "MoreThanOneFile"),
    (-120, "NodeIsInvalid", "Specified device was not found. Verify the device is connected and powered, and that the CAN bus is terminated."),
    (-121, "InvalidDeviceDescriptor", "InvalidDeviceDescriptor"),
    (-123, "CouldNotSendCanFrame", "CouldNotSendCanFrame"),
    (-124, "NormalModeMsgNotPresent", "NormalModeMsgNotPresent"),
    (-125, "FeatureNotSupported", "This feature is not supported."),
    (-126, "NotUpdating", "The diagnostic server is not field upgrading any devices."),
    (-127, "CorruptedPOST", "CorruptedPOST"),
    (-128, "NoConfigs", "This device did not report any available configs. Verify firmware and diagnostics are up-to-date."),
    (-129, "ConfigFailed", "ConfigFailed"),
    (-130, "CouldNotReqFactoryDefault", "Unable to factory default this device."),
    (-131, "CustomNameNotSupported", "CustomNameNotSupported"),
    (-132, "ConfigReadWriteMismatch", "The configs read from the device do not match the configs that were written."),
    (-133, "CouldNotReqSetConfigs", "Could not apply the device configs."),
    (-134, "InsufficientSz", "InsufficientSz"),
    (-135, "InvalidModel", "This feature is not supported for this device model."),
    (-140, "CouldNotReqDevInfo", "CouldNotReqDevInfo"),
    (-141, "NoControls", "This device does not support new controls."),
    (-142, "DeviceIsNull", "DeviceIsNull"),
    (-143, "DeviceDidNotRespondToDiagReq", "DeviceDidNotRespondToDiagReq"),
    (-144, "OnlySupportedInTunerX", "This feature requires Tuner X."),
    (-145, "CanivCliError", "Command-line issue with caniv."),
    (-200, "InvalidCrfBadHeader", "InvalidCrfBadHeader"),
    (-201, "InvalidCrfFileSzInvald", "InvalidCrfFileSzInvald"),
    (-202, "InvalidCrfWrongProduct", "Specified CRF is for the wrong product."),
    (-203, "InvalidCrfNoSects", "InvalidCrfNoSects"),
    (-204, "InvalidCrfBadSectHeader", "InvalidCrfBadSectHeader"),
    (-205, "InvalidCrfBadSectSize", "InvalidCrfBadSectSize"),
    (-206, "NoCrfFile", "Specified CRF file could not be found."),
    (-300, "CouldNotFindDynamicId", "CouldNotFindDynamicId"),
    (-301, "DidNotGetDhcp", "DidNotGetDhcp"),
    (-302, "DidNotGetFullDhcp", "DidNotGetFullDhcp"),
    (-350, "InvalidLicenseResp", "InvalidLicenseResp"),
    (-351, "InvalidCanivCache", "InvalidCanivCache"),
    (-500, "CannotOpenSerialPort", "CannotOpenSerialPort"),
    (-501, "CannotWriteSerialPort", "CannotWriteSerialPort"),
    (-502, "CannotReadSerialPort", "CannotReadSerialPort"),
    (-503, "CannotSerialToDevice", "CannotSerialToDevice"),
    (-504, "NoSerialControlFrameResp", "NoSerialControlFrameResp"),
    (-600, "CannotOpenUdpPort", "CannotOpenUdpPort"),
    (-601, "CannotWriteUdpPort", "CannotWriteUdpPort"),
    (-602, "CannotReadUdpPort", "CannotReadUdpPort"),
    (-603, "CannotUdpToDevice", "CannotUdpToDevice"),
    (-604, "NoUdpControlFrameResp", "NoUdpControlFrameResp"),
    (-605, "TimeoutIso15Response", "TimeoutIso15Response"),
    (-700, "InvalidJson", "InvalidJson"),
    (-800, "AppIsTerminating", "The user application is shutting down."),
    (1000, "CanMessageStale", "CAN message is stale, data is valid but old. Check the CAN bus wiring, CAN bus utilization, and power to the device."),
    (1006, "BufferFull", "Buffer is full, cannot insert more data."),
    (1010, "PulseWidthSensorNotPresent", "PulseWidthSensorNotPresent"),
    (1100, "GeneralWarning", "General Warning Occurred."),
    (1103, "FirmVersionCouldNotBeRetrieved", "Firm Vers could not be retrieved. Use Phoenix Tuner X to check ID and firmware(CRF) version."),
    (1104, "FeaturesNotAvailableYet", "This feature will be supported in a future update."),
    (1105, "ControlModeNotValid", "The control mode is not valid for this function."),
    (1106, "ControlModeNotSupportedYet", "This control mode is not supported yet.  A future release will supported this soon."),
    (1109, "MotProfFirmThreshold", "Motor Controller must have >= 3.2 firmware for motion profile control mode."),
    (1110, "MotProfFirmThreshold2", "Motor Controller must have >= 3.4 firmware for advanced PID0/PID1 features."),
    (1200, "SimDeviceNotFound", "SimDeviceNotFound"),
    (1201, "SimPhysicsTypeNotSupported", "SimPhysicsTypeNotSupported"),
    (1202, "SimDeviceAlreadyExists", "SimDeviceAlreadyExists"),
    (-1001, "TxFailed", "Could not transmit CAN Frame."),
    (-1002, "InvalidParamValue", "An invalid argument was passed into the function/VI, such as a null pointer."),
    (-1003, "RxTimeout", "CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device."),
    (-1004, "TxTimeout", "CAN Transmit timed out."),
    (-1005, "UnexpectedArbId", "ArbID is incorrect."),
    (-1006, "CanOverflowed", "CanOverflowed"),
    (-1007, "SensorNotPresent", "Sensor Not Present."),
    (-1008, "FirmwareTooOld", "Firmware Too Old.  Use Phoenix Tuner X to field upgrade your CTRE CAN device firmware(CRF).  Then restart your robot application to clear this error."),
    (-1009, "CouldNotChangePeriod", "Control Frame Period could not be changed.  Most likely it is not being transmitted."),
    (-1010, "BufferFailure", "BufferFailure"),
    (-1011, "FirmwareNonFRC", "Firmware is legacy non-FRC version.  Use Phoenix Tuner X to field upgrade your CTRE CAN device firmware(CRF).  Firmware greater than 20.0 required."),
    (-1100, "GeneralError", "General Error Occurred."),
    (-1200, "SigNotUpdated", "No new response to update signal."),
    (-1201, "NotAllPIDValuesUpdated", "NotAllPIDValuesUpdated"),
    (-1300, "GEN_PORT_ERROR", "GEN_PORT_ERROR"),
    (-1301, "PORT_MODULE_TYPE_MISMATCH", "PORT_MODULE_TYPE_MISMATCH"),
    (-1400, "GEN_MODULE_ERROR", "GEN_MODULE_ERROR"),
    (-1401, "MODULE_NOT_INIT_SET_ERROR", "MODULE_NOT_INIT_SET_ERROR"),
    (-1402, "MODULE_NOT_INIT_GET_ERROR", "MODULE_NOT_INIT_GET_ERROR"),
    (-1500, "WheelRadiusTooSmall", "Wheel Radius is too small, cannot get distance traveled."),
    (-1501, "TicksPerRevZero", "Ticks per revolution is 0, cannot get heading."),
    (-1502, "DistanceBetweenWheelsTooSmall", "Distance between wheels is too small, cannot get heading."),
    (-1503, "GainsAreNotSet", "GainsAreNotSet"),
    (-1504, "WrongRemoteLimitSwitchSource", "Use RemoteLimitSwitchSource instead of LimitSwitchSource."),
    (-1505, "DoubleVoltageCompensatingWPI", "Motor Controller Voltage Compensation should not be used with setVoltage().  This causes compensation to happen twice.  Disable Voltage Compensation by calling enableVoltageCompensation(false) in order to use setVoltage()."),
    (-1506, "CANdleAnimSlotOutOfBounds", "CANdleAnimSlotOutOfBounds"),
    (-1600, "IncompatibleMode", "IncompatibleMode"),
    (-1601, "InvalidHandle", "Handle passed into function is incorrect."),
    (-1700, "FeatureRequiresHigherFirm", "Features requires newer firmware version."),
    (-1702, "ConfigFactoryDefaultRequiresHigherFirm", "Config factory default features require firmware >=3.10."),
    (-1703, "ConfigMotionSCurveRequiresHigherFirm", "Config Motion S Curve Strength features require firmware >=4.16."),
    (-1704, "TalonFXFirmwarePreVBatDetect", "Talon FX(Falcon 500) Firmware Too Old.  Use Phoenix Tuner to field upgrade your CTRE CAN device firmware(CRF) to >=20.3. Then restart your robot application to clear this error."),
    (-1705, "CANdleAnimationsRequireHigherFirm", "CANdleAnimationsRequireHigherFirm"),
    (-1800, "LibraryCouldNotBeLoaded", "LibraryCouldNotBeLoaded"),
    (-1801, "MissingRoutineInLibrary", "MissingRoutineInLibrary"),
    (-1802, "ResourceNotAvailable", "ResourceNotAvailable"),
    (-1900, "MusicFileNotFound", "Could not find music file specified, try specifying an absolute path."),
    (-1901, "MusicFileWrongSize", "Music file size is incorrect, could not parse correctly. Ensure you're using Tuner to generate file."),
    (-1902, "MusicFileTooNew", "Music file version is too new, update Phoenix to utilize this file."),
    (-1903, "MusicFileInvalid", "Music file is invalid. Ensure you're using Tuner to generate file."),
    (-1904, "InvalidOrchestraAction", "An invalid orchestra action occurred. Ensure a music file is loaded."),
    (-1905, "MusicFileTooOld", "This music file version is too old. Regenerate file using Tuner."),
    (-1906, "MusicInterrupted", "Music interrupted due to one of the instruments being commanded a different control mode. Press Play to resume music."),
    (-1907, "MusicNotSupported", "This device doesn't support MusicTone control mode."),
    (-2000, "kInvalidInterface", "kInvalidInterface"),
    (-2001, "kInvalidGuid", "kInvalidGuid"),
    (-2002, "kInvalidClass", "kInvalidClass"),
    (-2003, "kInvalidProtocol", "kInvalidProtocol"),
    (-2004, "kInvalidPath", "kInvalidPath"),
    (-2005, "kGeneralWinUsbError", "kGeneralWinUsbError"),
    (-2006, "kFailedSetup", "kFailedSetup"),
    (-2007, "kListenFailed", "kListenFailed"),
    (-2008, "kSendFailed", "kSendFailed"),
    (-2009, "kReceiveFailed", "kReceiveFailed"),
    (-2010, "kInvalidRespFormat", "kInvalidRespFormat"),
    (-2011, "kWinUsbInitFailed", "kWinUsbInitFailed"),
    (-2012, "kWinUsbQueryFailed", "kWinUsbQueryFailed"),
    (-2013, "kWinUsbGeneralError", "kWinUsbGeneralError"),
    (-2014, "kAccessDenied", "kAccessDenied"),
    (-2015, "kFirmwareInvalidResponse", "kFirmwareInvalidResponse"),
    (-10000, "StatusCodeNotInitialized", "This StatusCode has not been initialized. Make sure the StatusCode is getting assigned to the return of a method."),
    (10000, "WarningNotInitialized", "WarningNotInitialized"),
    (10001, "HwTimestampOutOfSync", "The timestamp reported by CANivore is at least 10ms older than the timestamp reported by the system, indicating it's fallen out of sync. This does not impact the data of this message, only the timing."),
    (10002, "FrequentConfigCalls", "Do not apply or refresh configs periodically, as configs are blocking."),
    (-10001, "InvalidNetwork", "The given network/CAN bus is invalid. Verify that any status signals provided are on the same network and that the network is present."),
    (-10002, "MultiSignalNotSupported", "The CAN bus does not support multi-signal synchronization."),
    (-10003, "CouldNotCast", "Could not cast from base value to this particular signal's type"),
    (-10004, "NotFound", "Could not find this value when searching for it"),
    (-10005, "NotSupported", "This is not supported"),
    (-10006, "MissingContext", "Could not determine context from this device hash"),
    (-10007, "ModelMismatch", "Model name in license file does not match model name of selected device."),
    (-10008, "SerialMismatch", "Serial Number in license file does not match model name of selected device."),
    (-10009, "NoFile", "Could not find specified file."),
    (-10010, "LicenseDownloadFailed", "License did not successfully download to Device."),
    (-10011, "SelfTestIsEmpty", "Self Test report does not have any values, is the firmware up to date?"),
    (-10012, "SignalLookupFailed", "Failed to lookup signal properties.  This can happen if the fimware is too new and supports signals that older APIs do not support."),
    (-10013, "InvalidModeToGetSignal", "The current mode of the device is invalid for getting this signal."),
    (-10014, "UnlicensedDevice", "Device is not licensed. Cannot get any data from it."),
    (-10015, "InvalidSize", "Size is invalid."),
    (-10016, "InvalidLicenseResponse", "InvalidLicenseResponse"),
    (-10017, "InvalidContext", "InvalidContext"),
    (-10018, "InternalError", "InternalError"),
    (-10019, "kDeviceResponseIncorrect", "kDeviceResponseIncorrect"),
    (-10020, "kErrorPollingForDevices", "kErrorPollingForDevices"),
    (-10021, "CouldNotRetrieveV6Firmware", "Device firmware could not be retrieved. Check that the device is running v6 firmware, the device ID is correct, the specified CAN bus is correct, and the device is powered."),
    (-10022, "CouldNotDecodeDeviceFirmware", "Device firmware could not be decoded. Check that the device is running v6 firmware, the device ID is correct, the specified CAN bus is correct, and the device is powered."),
    (-10023, "InvalidIDToFollow", "The values specified for master are in valid.  Make sure the Device ID of master are correct."),
    (-10024, "UsingProFeatureOnUnlicensedDevice", "Using a Pro only feature on an unlicensed device. The device may not behave as expected if it continues to operate while unlicensed."),
    (-10025, "FirmwareTooNew", "Firmware Too New.  Use Phoenix Tuner X to field upgrade your CTRE CAN device firmware(CRF) to a compatible version.  Then restart your robot application to clear this error."),
    (-10026, "CouldNotSerialize", "The data frame could not be serialized for transmit."),
    (-10027, "MechanismFaulted", "The mechanism is disabled due to a fault in one of the devices."),
    (-10028, "FirmwareVersNotCompatible", "Firmware version is not compatible with this version of Phoenix. Make sure your firmware and API major versions match."),
    (-10029, "DirectoryMissing", "Could not find specified directory."),
    (-10030, "ApiTooOld", "This API version is too old for the firmware on the device. Either upgrade the API to a newer version or downgrade the device firmware to an older version for correct behavior."),
    (-10031, "LoggerNotRunning", "The signal logger is not running. Start the signal logger before writing any signals."),
    (-10032, "TimeoutCannotBeZero", "Blocking operations, such as configs, cannot have a timeout of 0. Pass in a non-zero timeout (typically 0.050+ seconds) for normal operation."),
    (-10033, "CannotLicenseWhileEnabled", "Device cannot be licensed while it is control enabled. Disable and neutral the device to apply the licenses."),
    (-10034, "InvalidFile", "Could not open or read the given file."),
    (-10035, "HootLogTooOld", "The given hoot log requires an older version of Phoenix API."),
    (-10036, "HootLogTooNew", "The given hoot log requires a newer version of Phoenix API."),
    (-10037, "UnlicensedHootLog", "Hoot log is not licensed. Cannot get any data from it."),
    (-10038, "CannotStepWhileUnpaused", "The simulation timing cannot be advanced by a time step while unpaused. Pause the simulator before advancing time."),
    (-10039, "MultipleReplayNotSupported", "Hoot replay does not support replaying multiple files. Ensure that only one file is loaded at a time."),
    (-10040, "UserSignalLimitExceeded", "The maximum number of loggable user signals has been exceeded. Additional user signals will not be logged."),
    (-10041, "InvalidDeviceModel", "The provided model was not a valid device type."),
    (-10042, "SignalNotSupportedOrMissing", "The requested signal is missing or not supported. Verify that the device firmware is up-to-date."),
    (-10043, "MissingSchema", "The schema for the logged user signal is invalid or missing. Ensure that the schema is written before the user signal."),
    (-10044, "InvalidDeviceId", "The device ID is invalid. Ensure the device is constructed with a valid ID (0-62)."),
];

impl StatusCode {
    pub const fn value(self) -> i32 {
        self.0
    }

    pub const fn is_ok(self) -> bool {
        self.0 == 0
    }

    pub const fn is_error(self) -> bool {
        self.0 < 0
    }

    pub const fn is_warning(self) -> bool {
        self.0 > 0
    }

    fn entry(self) -> Option<&'static (i32, &'static str, &'static str)> {
        TABLE.iter().find(|entry| entry.0 == self.0)
    }

    /// The CTRE name, such as `RxTimeout`.
    pub fn name(self) -> String {
        self.entry().map_or_else(
            || format!("Status Code {}", self.0),
            |entry| entry.1.to_string(),
        )
    }

    /// The CTRE description, such as "CAN frame not received/too-stale".
    pub fn description(self) -> String {
        self.entry().map_or_else(
            || format!("Status Code {}", self.0),
            |entry| entry.2.to_string(),
        )
    }
}

impl fmt::Display for StatusCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.name())
    }
}

impl fmt::Debug for StatusCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "StatusCode({}: {})", self.0, self.name())
    }
}

impl std::error::Error for StatusCode {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn table_has_every_header_code_once() {
        let mut seen = HashSet::new();
        for (value, ..) in TABLE {
            assert!(seen.insert(*value), "duplicate {value}");
        }
        assert_eq!(TABLE.len(), 188);
    }

    #[test]
    fn values_match_the_header() {
        // Read from ctre/phoenix/StatusCodes.h.
        assert_eq!(StatusCode::OK.value(), 0);
        assert_eq!(StatusCode::RxTimeout.value(), -1003);
        assert_eq!(StatusCode::StatusCodeNotInitialized.value(), -10000);
        assert_eq!(StatusCode::FirmwareTooOld.value(), -1008);
        assert_eq!(StatusCode::ApiTooOld.value(), -10030);
        assert_eq!(StatusCode::CouldNotRetrieveV6Firmware.value(), -10021);
        assert_eq!(StatusCode::InvalidNetwork.value(), -10001);
        assert_eq!(StatusCode::InvalidDeviceId.value(), -10044);
        assert_eq!(StatusCode::FrequentConfigCalls.value(), 10002);
        assert_eq!(StatusCode::TimeoutCannotBeZero.value(), -10032);
        assert_eq!(StatusCode::InvalidFile.value(), -10034);
        assert_eq!(StatusCode::HootLogTooOld.value(), -10035);
        assert_eq!(StatusCode::HootLogTooNew.value(), -10036);
        assert_eq!(StatusCode::UnlicensedHootLog.value(), -10037);
        assert_eq!(StatusCode::NotSupported.value(), -10005);
        assert_eq!(StatusCode::LoggerNotRunning.value(), -10031);
        assert_eq!(StatusCode::CanMessageStale.value(), 1000);
    }

    #[test]
    fn names_and_descriptions() {
        assert_eq!(StatusCode::OK.name(), "OK");
        assert_eq!(StatusCode(-1003).name(), "RxTimeout");
        assert_eq!(StatusCode(-1003).to_string(), "RxTimeout");
        assert!(
            StatusCode::RxTimeout
                .description()
                .contains("CAN frame not received")
        );
        assert_eq!(StatusCode::OK.description(), "No Error");
    }

    #[test]
    fn every_entry_name_roundtrips() {
        for (value, name, description) in TABLE {
            assert_eq!(StatusCode(*value).name(), *name);
            assert_eq!(StatusCode(*value).description(), *description);
        }
    }

    #[test]
    fn severity_follows_the_sign() {
        assert!(StatusCode::OK.is_ok());
        assert!(StatusCode::RxTimeout.is_error());
        assert!(!StatusCode::RxTimeout.is_ok());
        assert!(StatusCode::CanMessageStale.is_warning());
        assert!(!StatusCode::CanMessageStale.is_error());
    }

    #[test]
    fn unknown_codes_keep_their_number() {
        assert_eq!(StatusCode(12345).name(), "Status Code 12345");
        assert_eq!(StatusCode(12345).value(), 12345);
        assert!(StatusCode(12345).is_warning());
    }
}
