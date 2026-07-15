package com.example.robotweb.repository;

import com.example.robotweb.entity.RobotStatus;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface RobotStatusRepository extends JpaRepository<RobotStatus, Long> {

    Optional<RobotStatus> findTopByOrderByUpdatedAtDesc();
}