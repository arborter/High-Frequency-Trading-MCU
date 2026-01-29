/*

Abstract:
* 	The hedge-fund's process for a decision on a stock
* 	in a High Frequency Trading environment.

*/





/*	1.) data organization layer	*/

typedef struct {
	double price;
	int volume;
	double volatility;
	uint64_t ts_ns;
} Market_Tick;

typedef struct {
	double last_price;       // historic data
	int last_volume;
	
	double ema_price;        // exponential moving average
	double ema_volatility;
	
	double variance;         // running variance estimate
	double stddev;
	
	uint64_t last_ts_ns;     // authoritative market time
	uint64_t tick_count;
} Stock_Delta;






/*	 2.) stock characteristics layer	*/
void stock_initialization(Stock_Delta *f){]
	memset(f, 0, sizeof(*f));
}

void stock_update(Stock_Delta *f, const Market_Tick *t) {
    const double alpha = 0.05;

    if (f->tick_count == 0) {
        f->ema_price = t->price;
        f->ema_volatility = t->volatility;
        f->variance = 0.0;
    } else {
        double diff = t->price - f->ema_price;

        f->ema_price = alpha * t->price + (1.0 - alpha) * f->ema_price;
        f->ema_volatility = alpha * t->volatility +
                           (1.0 - alpha) * f->ema_volatility;

        f->variance = alpha * (diff * diff) +
                      (1.0 - alpha) * f->variance;
    }

    f->stddev = sqrt(f->variance);
    f->last_price = t->price;
    f->last_volume = t->volume;
    f->last_ts_ns = t->ts_ns;
    f->tick_count++;
}






/*	 3.) Stratgy Layer	*/

typedef enum {
	SIGNAL_BUY = 1;
	SIGNAL_SELL = - 1;
	SIGNAL_HOLD = 0;
} TradeSignal;


TradeSignal strategy_decide(const Stock_Delta *f) {
    if (f->tick_count < 10)
        return SIGNAL_HOLD;

    if (f->last_price > f->ema_price &&
        f->ema_volatility < 0.3)
        return SIGNAL_BUY;

    if (f->last_price < f->ema_price)
        return SIGNAL_SELL;

    return SIGNAL_HOLD;
}






/*	 4.) Logging of final decision per tick via MQTT */
void on_market_tick(const Market_Tick *t) {
    stock_update(&some_stock, t);

    TradeSignal sig = strategy_decide(&some_stock);

    if (sig != SIGNAL_HOLD)
        publish_signal(sig); // MQTT call to publish on the Pi Broker log.
}

