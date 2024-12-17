#pragma once

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_combined.h>

namespace uavcannode
{

class RawIMU :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>
{
public:
	RawIMU(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::ahrs::RawIMU::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_combined)),
		uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>(node)
	{
		this->setPriority(uavcan::TransferPriority::Default);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::ahrs::RawIMU::getDataTypeFullName(),
			       uavcan::equipment::ahrs::RawIMU::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		sensor_combined_s meas;

		if (uORB::SubscriptionCallbackWorkItem::update(&meas)) {
			uavcan::equipment::ahrs::RawIMU msg{};

			// Only these fields are actually used by PX4 sensor bridge,
			// ignoring the rest.
			msg.rate_gyro_latest[0] = meas.gyro_rad[0];
			msg.rate_gyro_latest[1] = meas.gyro_rad[1];
			msg.rate_gyro_latest[2] = meas.gyro_rad[2];

			msg.accelerometer_latest [0] = meas.accelerometer_m_s2[0];
			msg.accelerometer_latest [1] = meas.accelerometer_m_s2[1];
			msg.accelerometer_latest [2] = meas.accelerometer_m_s2[2];

			uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>::broadcast(msg);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
}
