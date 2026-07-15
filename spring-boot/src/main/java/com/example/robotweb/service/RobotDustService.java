package com.example.robotweb.service;

import org.springframework.stereotype.Service;

/**
 * MQTT robot/1/dust 의 가장 최근 페이로드(JSON 문자열)를 메모리에 보관한다.
 * dust_mapping/dust_mapper 가 이미 ready-to-display JSON 을 만들어서 보내므로
 * Spring 쪽에서는 파싱하지 않고 그대로 들고 있다가 /api/robot/dust 가 호출되면
 * 그대로 응답으로 흘려준다.
 */
@Service
public class RobotDustService {

    private volatile String latestPayload = null;

    public void update(String payload) {
        this.latestPayload = payload;
    }

    public String getLatestPayload() {
        return latestPayload;
    }
}
