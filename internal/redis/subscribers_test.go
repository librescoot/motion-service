package redis

import (
	"log/slog"
	"testing"
)

func TestSensorRateForVehicleState(t *testing.T) {
	for state, want := range map[string]int{
		"parked":               FastPollRateHz,
		"ready-to-drive":       FastPollRateHz,
		"stand-by":             IdlePollRateHz,
		"hibernating":          IdlePollRateHz,
		"hibernating-imminent": IdlePollRateHz,
		"hibernating-waiting":  IdlePollRateHz,
		"hop-on":               SlowPollRateHz,
		"hop-on-learning":      SlowPollRateHz,
		"waiting-seatbox":      SlowPollRateHz,
		"shutting-down":        SlowPollRateHz,
		"":                     SlowPollRateHz,
		"something-new":        SlowPollRateHz,
	} {
		if got := SensorRateForVehicleState(state); got != want {
			t.Errorf("state %q: rate %d, want %d", state, got, want)
		}
	}
}

type recordedRate struct{ rates []int }

func (r *recordedRate) SetRate(hz int) { r.rates = append(r.rates, hz) }

func TestVehicleStateSuspendsAndResumesPolling(t *testing.T) {
	rate := &recordedRate{}
	s := &Subscriber{rate: rate, log: slog.Default()}
	for _, state := range []string{"stand-by", "parked", "ready-to-drive", "parked", "shutting-down", "stand-by"} {
		if err := s.handleVehicleState(state); err != nil {
			t.Fatal(err)
		}
	}
	want := []int{0, 5, 5, 5, 1, 0}
	if len(rate.rates) != len(want) {
		t.Fatalf("rates %v, want %v", rate.rates, want)
	}
	for i := range want {
		if rate.rates[i] != want[i] {
			t.Fatalf("rates %v, want %v", rate.rates, want)
		}
	}
}
