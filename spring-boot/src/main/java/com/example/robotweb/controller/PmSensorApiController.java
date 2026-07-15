package com.example.robotweb.controller;

import com.example.robotweb.entity.PmSensorData;
import com.example.robotweb.repository.PmSensorDataRepository;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/sensor/pm")
public class PmSensorApiController {

    private final PmSensorDataRepository pmSensorDataRepository;

    public PmSensorApiController(PmSensorDataRepository pmSensorDataRepository) {
        this.pmSensorDataRepository = pmSensorDataRepository;
    }

    @GetMapping("/latest")
    public PmSensorData getLatestData() {
        return pmSensorDataRepository.findTopByOrderByCreatedAtDesc()
                .orElse(new PmSensorData("zoneA", 0, 0, 0, 0));
    }

    @PostMapping
    public PmSensorData saveData(@RequestBody PmSensorData pmSensorData) {
        PmSensorData newData = new PmSensorData(
                pmSensorData.getZone(),
                pmSensorData.getPm1_0(),
                pmSensorData.getPm2_5(),
                pmSensorData.getPm10(),
                pmSensorData.getRssi()
        );

        return pmSensorDataRepository.save(newData);
    }
}