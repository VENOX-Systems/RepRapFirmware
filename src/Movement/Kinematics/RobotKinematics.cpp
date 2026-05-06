/*
 * RobotKinematics.cpp
 *
 *  Created on: 06 Aug 2023
 *      Author: JoergS5
 */


#include <Movement/Kinematics/RobotKinematics.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Movement/DDA.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RobotKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry RobotKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
	//TODO lots more to be added here
};

constexpr uint8_t RobotKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(RobotKinematics, ZLeadscrewKinematics)

#endif


RobotKinematics::RobotKinematics() noexcept
	: ZLeadscrewKinematics(KinematicsType::scara, SegmentationType(true, false, false))
{
}

RobotKinematics::~RobotKinematics() {
}

const char *RobotKinematics::GetName(bool forStatusReport) const noexcept {
	return "robot";
}

bool RobotKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply,
		bool& error) THROWS(GCodeException) {

	if (mCode == 669)
	{
		bool seen = false;
		bool allowEmpty = false;
		bool letterUsed = false;

		if (gb.Seen('B')) {
			gb.TryGetQuotedString('B', tempS50.GetRef(), seen, allowEmpty);
			setB(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('A')) {
			gb.TryGetQuotedString('A', tempS50.GetRef(), seen, allowEmpty);
			setA(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('P')) {
			gb.TryGetQuotedString('P', tempS50.GetRef(), seen, allowEmpty);
			setP(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('C')) {
			gb.TryGetQuotedString('C', tempS50.GetRef(), seen, allowEmpty);
			setC(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('R')) {
			float avgTime = sumOfTimes / (float) timeMeasurements;
			tempS50.Clear();
			tempS50.catf("avg Time inverse kin: %.2f microseconds", (double) avgTime);
			consoleMessage(tempS50.GetRef());
			tempS50.copy("\n");
			consoleMessage(tempS50.GetRef());
			sumOfTimes = 0;
			timeMeasurements = 0;
			letterUsed = true;
		}

		// set not to unhome
		if (gb.Seen('D')) {
			seen = false; // don't unhome
			letterUsed = true;
		}

		if(!letterUsed) {
			reportConfiguration(gb);
		}

		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

// Skip the non-linear robot IK while any kinematic axis is still unhomed. The IK couples motors
// 0/1/2 to A and C; running it with bogus angles drags the carriage motors when homing A/C.
// While unhomed, fall back to a pure CoreXY mix on X/Y plus pass-through on Z/A/C so each
// homing move drives only the motors expected for that axis.
bool RobotKinematics::UseLinearKinematics() const noexcept {
	const GCodes& gc = reprap.GetGCodes();
	const int axesToCheck = min<int>(numOfAxes, (int)MaxAxes);
	for (int axis = 0; axis < axesToCheck; ++axis) {
		if (!gc.IsAxisHomed(axis)) {
			return true;
		}
	}
	return false;
}

MovementError RobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept {


	if(specialMethod == 1) { // CoreXY
		if (UseLinearKinematics()) {
			const float x = machinePos[0];
			const float y = machinePos[1];
			motorPos[0] = lrintf((x + y) * stepsPerMm[0]);
			motorPos[1] = lrintf((x - y) * stepsPerMm[1]);
			motorPos[2] = lrintf(machinePos[2] * stepsPerMm[2]);
			motorPos[3] = lrintf(machinePos[3] * stepsPerMm[3]);
			motorPos[4] = lrintf(machinePos[4] * stepsPerMm[4]);
		}
		else {
			int i = -1;
			bool isAC = true;
			if(abcType == 0) {
				i = getPositionOfLetterInChain('A');
				isAC = true;
			}
			else if(abcType == 1) {
				i = getPositionOfLetterInChain('B');
				isAC = false;
			}
			const float ABpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};
			i = getPositionOfLetterInChain('C');
			const float Cpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};

			float result[5];
			GACartesianToMotor(machinePos, ABpt, Cpt, result, isAC);

			const float coreX = result[0] + result[1];
			const float coreY = result[0] - result[1];
			motorPos[0] = lrintf(coreX * stepsPerMm[0]); // X (A stepper)
			motorPos[1] = lrintf(coreY * stepsPerMm[1]); // Y (B stepper)
			motorPos[2] = lrintf(result[2] * stepsPerMm[2]); // Z
			motorPos[3] = lrintf(machinePos[3] * stepsPerMm[3]); // A/B
			motorPos[4] = lrintf(machinePos[4] * stepsPerMm[4]); // C
		}
	}

	for (size_t axis = numOfAxes; axis < numVisibleAxes; ++axis)	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}


	return MovementError::ok;
}

//called after connection established, called once for every segment move:

void RobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept {
	startClock();

	if(specialMethod == 1) { // CoreXY
		const float m0 = (float) motorPos[0] / stepsPerMm[0];
		const float m1 = (float) motorPos[1] / stepsPerMm[1];

		if (UseLinearKinematics()) {
			machinePos[0] = 0.5f * (m0 + m1);
			machinePos[1] = 0.5f * (m0 - m1);
			machinePos[2] = (float) motorPos[2] / stepsPerMm[2];
			machinePos[3] = (float) motorPos[3] / stepsPerMm[3];
			machinePos[4] = (float) motorPos[4] / stepsPerMm[4];
		}
		else {
			float angles[numOfAxes];
			angles[0] = 0.5f * (m0 + m1);
			angles[1] = 0.5f * (m0 - m1);
			angles[2] = (float) motorPos[2] / stepsPerMm[2]; // Z
			angles[3] = (float) motorPos[3] / stepsPerMm[3]; // A or B
			angles[4] = (float) motorPos[4] / stepsPerMm[4]; // C

			int i = -1;
			bool isAC = true;
			if(abcType == 0) {
				i = getPositionOfLetterInChain('A');
				isAC = true;
			}
			else if(abcType == 1) {
				i = getPositionOfLetterInChain('B');
				isAC = false;
			}
			const float ABpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};
			i = getPositionOfLetterInChain('C');
			const float Cpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};

			GAMotorToCartesian(angles, ABpt, Cpt, machinePos, isAC);
		}
	}

	sumOfTimes += stopClock();
	timeMeasurements++;

}

bool RobotKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept {
	return true;
}

LimitPositionResult RobotKinematics::LimitPosition(float coords[], const float * null initialCoords,
		size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept {

//	const bool m208Limited = applyM208Limits && Kinematics::LimitPositionFromAxis(coords, 0, numVisibleAxes, axesToLimit);
//	return (m208Limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;

	return LimitPositionResult::ok;
}

void RobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept {

}

AxesBitmap RobotKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept {
	return g92Axes;
}

AxesBitmap RobotKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept {
	return axesMoving;
}


AxesBitmap RobotKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed,
		size_t numVisibleAxes, const StringRef& filename) const noexcept {
	AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	filename.copy(HomeRobotFileName);
	return ret;

}

bool RobotKinematics::IsContinuousRotationAxis(size_t axis) const noexcept {
	return false;
}

// In CoreXY mode, motors 0 and 1 are both involved in moving axes X and Y. Report that so
// stall-detect homing enables both motors and movement allocation reserves the right drives.
LogicalDrivesBitmap RobotKinematics::GetControllingDrives(size_t axis, bool forHoming) const noexcept {
	if (specialMethod == 1 && (axis == 0 || axis == 1)) {
		LogicalDrivesBitmap result;
		result.SetBit(0);
		result.SetBit(1);
		return result;
	}
	return Kinematics::GetControllingDrives(axis, forHoming);
}

// Used by the firmware on homeCartesianAxes machines to translate per-axis homing speeds
// into per-motor speeds. CoreXY: m0 = X + Y, m1 = X - Y. Other axes pass through unchanged.
void RobotKinematics::ConvertAxisAmountsToLogicalDriveAmounts(float amounts[MaxAxes], size_t numVisibleAxes, size_t numTotalAxes) const noexcept {
	if (specialMethod == 1) {
		const float x = amounts[0];
		const float y = amounts[1];
		amounts[0] = x + y;
		amounts[1] = x - y;
	}
}

void RobotKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector,
		size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept {

}
