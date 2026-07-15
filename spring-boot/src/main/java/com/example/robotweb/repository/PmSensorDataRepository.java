package com.example.robotweb.repository;

import com.example.robotweb.entity.PmSensorData;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface PmSensorDataRepository extends JpaRepository<PmSensorData, Long> {

    Optional<PmSensorData> findTopByOrderByCreatedAtDesc();
}