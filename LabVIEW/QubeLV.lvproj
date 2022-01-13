<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="20008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Qube" Type="Folder" URL="../Qube">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="Main.vi" Type="VI" URL="../Main.vi"/>
		<Item Name="MainCascaded.vi" Type="VI" URL="../MainCascaded.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="AnalogInputs.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/AnalogInputs.ctl"/>
				<Item Name="AnalogOutputs.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/AnalogOutputs.ctl"/>
				<Item Name="DigitalIO.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/DigitalIO.ctl"/>
				<Item Name="EncoderInputs.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/EncoderInputs.ctl"/>
				<Item Name="Error Cluster From Error Code.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Error Cluster From Error Code.vi"/>
				<Item Name="HardwareClocks.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/HardwareClocks.ctl"/>
				<Item Name="HIL Board.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Board/HIL Board.lvclass"/>
				<Item Name="HIL Task.lvclass" Type="LVClass" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Task/HIL Task.lvclass"/>
				<Item Name="OtherOutputs.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/OtherOutputs.ctl"/>
				<Item Name="Poly HIL Close.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Board/_Menus/Poly HIL Close.vi"/>
				<Item Name="PWMOutputs.ctl" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/PWMOutputs.ctl"/>
				<Item Name="Quanser RCP Utilities.lvlib" Type="Library" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/utilities/Quanser RCP Utilities.lvlib"/>
				<Item Name="subExtractAnalogInputRanges.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractAnalogInputRanges.vi"/>
				<Item Name="subExtractAnalogOutputExpirationStates.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractAnalogOutputExpirationStates.vi"/>
				<Item Name="subExtractAnalogOutputRanges.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractAnalogOutputRanges.vi"/>
				<Item Name="subExtractClockModes.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractClockModes.vi"/>
				<Item Name="subExtractDigitalParameters.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractDigitalParameters.vi"/>
				<Item Name="subExtractEncoderFilterFrequencies.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractEncoderFilterFrequencies.vi"/>
				<Item Name="subExtractEncoderQuadratureModes.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractEncoderQuadratureModes.vi"/>
				<Item Name="subExtractFinalAnalogOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractFinalAnalogOutputs.vi"/>
				<Item Name="subExtractFinalOtherOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractFinalOtherOutputs.vi"/>
				<Item Name="subExtractFinalPWMOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractFinalPWMOutputs.vi"/>
				<Item Name="subExtractInitialAnalogOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractInitialAnalogOutputs.vi"/>
				<Item Name="subExtractInitialClockFrequencies.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractInitialClockFrequencies.vi"/>
				<Item Name="subExtractInitialEncoderCounts.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractInitialEncoderCounts.vi"/>
				<Item Name="subExtractInitialOtherOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractInitialOtherOutputs.vi"/>
				<Item Name="subExtractInitialPWMOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractInitialPWMOutputs.vi"/>
				<Item Name="subExtractOtherOutputExpirationStates.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractOtherOutputExpirationStates.vi"/>
				<Item Name="subExtractPWMConfigurations.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractPWMConfigurations.vi"/>
				<Item Name="subExtractPWMDeadbands.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractPWMDeadbands.vi"/>
				<Item Name="subExtractPWMModes.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractPWMModes.vi"/>
				<Item Name="subExtractPWMOutputExpirationStates.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subExtractPWMOutputExpirationStates.vi"/>
				<Item Name="subHIL Initialize.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subHIL Initialize.vi"/>
				<Item Name="subInitializeAnalogInputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializeAnalogInputs.vi"/>
				<Item Name="subInitializeAnalogOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializeAnalogOutputs.vi"/>
				<Item Name="subInitializeClocks.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializeClocks.vi"/>
				<Item Name="subInitializeDigitalIO.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializeDigitalIO.vi"/>
				<Item Name="subInitializeEncoderInputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializeEncoderInputs.vi"/>
				<Item Name="subInitializeOtherOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializeOtherOutputs.vi"/>
				<Item Name="subInitializePWMOutputs.vi" Type="VI" URL="/&lt;vilib&gt;/Quanser/Rapid Control Prototyping Toolkit/hardware/HIL Initialize/HIL InitializeSource.llb/subInitializePWMOutputs.vi"/>
			</Item>
			<Item Name="complimentaryFilter.vi" Type="VI" URL="../../../../../Desktop/complimentaryFilter.vi"/>
			<Item Name="hil.framework" Type="Document" URL="/&lt;resource&gt;/hil.framework"/>
			<Item Name="lowpas.vi" Type="VI" URL="../../../../../Desktop/lowpas.vi"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
