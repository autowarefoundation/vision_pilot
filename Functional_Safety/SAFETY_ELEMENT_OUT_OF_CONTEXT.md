# Vision Pilot Safety Element out of Context (SEooC) Scope Definition & ISO 26262 Compliance Boundary - DRAFT

## 1. Purpose

### Objective

Define the safety scope of the Autoware Vision Pilot ADAS software SEooC and establish:
- Included software functions
- Excluded functionality and responsibilities
- Assumptions of use
- External dependencies
- Intended ISO 26262 compliance targets

### Key Message
Deliver an ISO 26262-compliant software SEooC based on selected Autoware components, with compliance focused on ISO 26262 Part 6 and applicable supporting processes from Part 8, while leaving vehicle-level safety activities and integration responsibilities to the OEM or Tier-1 integrator.

## 2. SEooC Scope

### SEooC Description

#### Autoware ADAS Software SEooC

The SEooC consists of a selected subset of the Autoware Vision Pilot open-source software stack providing reusable software functions for camera-based ADAS applications. The scope includes planning, vehicle motion control, and the Safety Guardian software components that can be integrated into an OEM or Tier-1 system architecture.

The SEooC is provided independently of a specific vehicle platform, sensor implementation, hardware platform, or system architecture. Assumptions, constraints, and interface requirements are defined to support safe integration by the system integrator.
The objective of the safety activities is to demonstrate compliance of the included software components with ISO 26262 Part 6 and applicable supporting processes from Part 8.

### Characteristics
- Software-only scope
- Open-source based
- Platform independent
- Vehicle independent
- Intended for integration by OEMs and Tier-1 suppliers

**This SEooC does not constitute a complete vehicle system**

### 3. Included Functional Scope

#### Functions Included in the SEooC
**Safety Guardian Functions**
- Fusing and tracking the outputs of AI networks in Vision Pilot 
- Calculating a safe driving plan for in-lane driving 

**ADAS Functions**
- Lane Departure Warning (LDW)
- Forward Collision Warning (FCW)

**These functions are provided at software level as reusable building blocks.**

## 4. SEooC Boundary

### Outside the SEooC
These remain OEM/Tier-1 responsibilities.

- Perception Software
- AI models
- Vehicle hardware
- HPC platform
- Steering actuator ECU
- Brake actuator ECU
- Powertrain controller
- Driver monitoring system
- Vehicle HMI implementation
- Sensor hardware
- Vehicle integration
- Production validation
- Homologation activities
- Final safety case

## 5. External Interfaces

### Inputs
- Front camera perception data 
- Vehicle speed 
- Steering angle 
- Vehicle status information

### Outputs
- Steering targets 
- Acceleration targets 
- Deceleration/brake targets 
- Diagnostic status 
- Warning requests

### Communication
- Interface technology is implementation dependent. 
- CAN is considered an example integration mechanism.


## 6. Assumption of Use

### The SEooC assumes:
**Violation of these assumptions requires mitigation by the system integrator.**

- Valid camera calibration
- Operational sensor hardware
- Clear visibility of the road scene by the camera
- Accurate perception of objects and road elements in the scene by upstream perception components
- Reliable communication infrastructure
- Functional vehicle actuators
- Sufficient computing resources
- Driver supervision according to SAE Level 2
- OEM implementation of driver monitoring and warning strategies

## 7. Known Limitations

### Architectural Limitations
**These limitations must be considered by the integrator safety concept.**
- No sensor redundancy assumed

## 8. Intended ISO 26262 Compliance Scope

### The SEooC is intended to comply with:

#### ISO 26262 Part 6
- Product Development at the Software Level
- Software safety requirements
- Software architectural design
- Unit design and implementation
- Software integration and testing
- Verification activities

#### Relevant ISO 26262 Part 8 Supporting Processes
- Configuration Management
- Change Management
- Verification
- Documentation Management
- Qualification of Software Tools (when applicable)
- Confidence in Use of Software Components
- Proven-in-Use Arguments (if applicable)
- Interface Management

#### Not Intended to be Covered by the SEooC:
- ISO 26262 Part 2 - Functional Safety Management
- ISO 26262 Part 3 - Concept Phase activities supporting SEooC assumptions
- ISO 26262 Part 4 - System Development 
- ISO 26262 Part 5 - Hardware Development

**These activities remain the responsibility of the OEM/Tier-1 integrator. Some work items in them may be available as a starting point for the integrator to add on them.**

## 9. Future Safety Roadmap

### Beyond ISO 26262

**Future activities may include:**

#### ISO 21448 (SOTIF)
- AI perception limitations
- ODD definition
- Scenario-based validation

#### ISO/PAS 8800
- Safety and AI governance
- AI development lifecycle
- AI assurance arguments

**These activities complement functional safety and address performance-related safety risks.**

## 10. Key Takeaway

### Compliance Goal
The objective is not to certify a complete ADAS system. The objective is to provide an ISO 26262-oriented software SEooC that:

- Delivers reusable ADAS software functionality
- Defines clear assumptions and interfaces
- Enables OEM/Tier-1 integration
- Supports downstream vehicle-level safety activities

**This scope definition establishes the boundary for all subsequent safety work products and compliance activities.**








