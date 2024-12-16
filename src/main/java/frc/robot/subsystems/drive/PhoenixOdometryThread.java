// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import frc.robot.util.DriveConstants;
import frc.robot.util.TunerConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class PhoenixOdometryThread extends Thread
{
	private final Lock signalsLock = new ReentrantLock();
	private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
	private final List<DoubleSupplier> genericSignals = new ArrayList<>();
	private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
	private final List<Queue<Double>> genericQueues = new ArrayList<>();
	private final List<Queue<Double>> timestampQueues = new ArrayList<>();
	private static boolean isCANFD = false;// = new CANBus(ZippyConstants.TunerConstants.DrivetrainConstants.CANBusName)
	// .isNetworkFD();
	private static boolean hasCANInitialized = false;

	private static PhoenixOdometryThread instance = null;

	public static PhoenixOdometryThread getInstance()
	{
		if (instance == null)
		{
			instance = new PhoenixOdometryThread();
		}
		return instance;
	}

	private PhoenixOdometryThread()
	{
		setName("PhoenixOdometryThread");
		setDaemon(true);
	}

	@Override
	public void start()
	{
		if (timestampQueues.size() > 0)
		{
			super.start();
		}
	}

	public Queue<Double> registerSignal(StatusSignal<Angle> signal)
	{
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		signalsLock.lock();
		Drive.odometryLock.lock();
		try
		{
			BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
			System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
			newSignals[phoenixSignals.length] = signal;
			phoenixSignals = newSignals;
			phoenixQueues.add(queue);
		} finally
		{
			signalsLock.unlock();
			Drive.odometryLock.unlock();
		}
		return queue;
	}

	public Queue<Double> registerSignal(DoubleSupplier signal)
	{
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		signalsLock.lock();
		Drive.odometryLock.lock();
		try
		{
			genericSignals.add(signal);
			genericQueues.add(queue);
		} finally
		{
			signalsLock.unlock();
			Drive.odometryLock.unlock();
		}
		return queue;
	}

	public Queue<Double> makeTimestampQueue()
	{
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		Drive.odometryLock.lock();
		try
		{
			timestampQueues.add(queue);
		} finally
		{
			Drive.odometryLock.unlock();
		}
		return queue;
	}

	public void run(DriveConstants driveConstants, TunerConstants tunerConstants)
	{
		if (!hasCANInitialized)
		{
			hasCANInitialized = true;
			isCANFD = new CANBus(tunerConstants.DrivetrainConstants().CANBusName).isNetworkFD();
			return;
		}
		while (true)
		{
			signalsLock.lock();
			try
			{
				if (isCANFD && phoenixSignals.length > 0)
				{
					BaseStatusSignal.waitForAll(2.0 / driveConstants.odometryFrequency(), phoenixSignals);
				} else
				{
					Thread.sleep((long) (1000.0 / driveConstants.odometryFrequency()));
					if (phoenixSignals.length > 0)
						BaseStatusSignal.refreshAll(phoenixSignals);
				}
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			} finally
			{
				signalsLock.unlock();
			}

			Drive.odometryLock.lock();
			try
			{

				double timestamp = RobotController.getFPGATime() / 1e6;
				double totalLatency = 0.0;
				for (BaseStatusSignal signal : phoenixSignals)
				{
					totalLatency += signal.getTimestamp().getLatency();
				}
				if (phoenixSignals.length > 0)
				{
					timestamp -= totalLatency / phoenixSignals.length;
				}

				for (int i = 0; i < phoenixSignals.length; i++)
				{
					phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
				}
				for (int i = 0; i < genericSignals.size(); i++)
				{
					genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
				}
				for (int i = 0; i < timestampQueues.size(); i++)
				{
					timestampQueues.get(i).offer(timestamp);
				}
			} finally
			{
				Drive.odometryLock.unlock();
			}
		}
	}
}