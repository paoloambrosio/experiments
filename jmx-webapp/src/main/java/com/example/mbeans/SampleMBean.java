package com.example.mbeans;

import org.springframework.jmx.export.annotation.ManagedAttribute;
import org.springframework.jmx.export.annotation.ManagedResource;
import org.springframework.stereotype.Component;

@Component
@ManagedResource
public class SampleMBean {

	@ManagedAttribute
    public String getAnAttribute() {
        return "Some value";
    }
}