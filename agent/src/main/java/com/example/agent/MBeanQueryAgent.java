package com.example.agent;

import java.lang.instrument.Instrumentation;
import java.lang.management.ManagementFactory;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.management.MBeanServer;
import javax.management.ObjectName;

public class MBeanQueryAgent {

	private final static int TICK = 10;

	public static void premain(String args, Instrumentation inst) {
		System.out.println("AGENT == Agent starting");

		final MBeanServer mbeanServer = ManagementFactory.getPlatformMBeanServer();

		final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();

		Runtime.getRuntime().addShutdownHook(new Thread() {
			@Override
			public void run() {
				System.out.println("AGENT == Agent shutting down");
				scheduler.shutdown();
			}
		});

		scheduler.scheduleWithFixedDelay(new Runnable() {
			public void run() {
				try {
					ObjectName objectName = new ObjectName("com.example.mbeans:name=sampleMBean,type=SampleMBean");
					String attributeName = "AnAttribute";
					Object attribute = mbeanServer.getAttribute(objectName, attributeName);
					System.out.println("AGENT == Bean attribute is " + attribute);
				} catch (Exception e) {
					System.out.println("AGENT == Exception " + e.getMessage());
				}
			}
		}, TICK, TICK, TimeUnit.SECONDS);

	}

}