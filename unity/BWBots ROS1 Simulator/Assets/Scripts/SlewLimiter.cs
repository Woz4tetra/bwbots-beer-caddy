using System;
using UnityEngine;

public class SlewLimiter
{
    float m_AccelLimit = 1.0f;
    float m_DecelLimit = 1.0f;
    float m_PrevValue = 0.0f;
    float m_PrevTime = 0.0f;
    public SlewLimiter(float deceleration_limit, float acceleration_limit, float initial_value)
    {
        m_AccelLimit = Math.Abs(acceleration_limit);
        m_DecelLimit = -Math.Abs(deceleration_limit);
        m_PrevTime = 0.0f;
        m_PrevValue = initial_value;
    }

    public static float clamp(float value, float lower, float upper) {
        return Math.Max(lower, Math.Min(value, upper));
    }

    public float calculate(float input) {
        float current_time = getTime();
        float dt = current_time - m_PrevTime;
        m_PrevTime = current_time;
        m_PrevValue += clamp(input - m_PrevValue, m_DecelLimit * dt, m_AccelLimit * dt);
        return m_PrevValue;
    }

    public float getTime() {
        return (float)Time.timeAsDouble;
    }

    public void reset(float value) {
        m_PrevValue = value;
        m_PrevTime = getTime();
    }
}