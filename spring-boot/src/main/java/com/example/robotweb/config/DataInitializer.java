package com.example.robotweb.config;

import com.example.robotweb.entity.RobotStatus;
import com.example.robotweb.repository.RobotStatusRepository;
import org.springframework.boot.CommandLineRunner;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class DataInitializer {

    @Bean
    CommandLineRunner initData(RobotStatusRepository robotStatusRepository) {
        return args -> {
            if (robotStatusRepository.count() == 0) {
                RobotStatus robotStatus = new RobotStatus(
                        "TurtleBot3",
                        "대기중",
                        85,
                        12
                );

                robotStatusRepository.save(robotStatus);
            }
        };
    }
}