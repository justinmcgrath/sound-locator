library(serial)
library(lattice)

con = serialConnection(name = "arduino",
                        port = "COM10",
                        mode = "115200,n,8,1",
                        buffering = "none",
                        newline = 1,
                        translation = "cr")

delay = function(x, y) {
   max_delay = 100
   max_correlation = 0;
   shift_at_max_correlation = -max_delay;
   for (shift in seq(-max_delay, max_delay)) { # shift = -max_delay; shift <= max_delay; ++shift) {
        if (shift < 0) {
          front = -seq(0, shift)[-1]
          back = seq(length(y) + shift, length(y) - 1)
          tx = x[-front]
          ty = y[-back]
        }
        else if (shift > 0) {
          front = -seq(0, shift)[-1]
          back = seq(length(y) - shift + 1, length(y))
          tx = x[-back]
          ty = y[front]
        } else {
          tx = x
          ty = y
        }
print(shift)
        r = cor(tx, ty)
        print(r)
        if (abs(r) > abs(max_correlation)) {
    print("UPDATING")
            max_correlation = r;
            shift_at_max_correlation = shift;
        }
    }
    return(shift_at_max_correlation)
}

with(subset(foo, time > 56800 & time < 57000), delay(foo$mic1, foo$mic2))



do_read = function(run_time) {
        open(con)

        stopTime = Sys.time() + run_time
        foo = data.frame(time=NA, mic1=NA, mic2=NA)

        last_plot = Sys.time()
        read.serialConnection(con)
        read.serialConnection(con)
        while(Sys.time() < stopTime)
        {
            newText = read.serialConnection(con)
            if(0 < nchar(newText))
            {
                temp = read.csv(textConnection(newText), header=FALSE, col.names=c('time', 'mic1', 'mic2'))
                temp = within(temp, {
                        time = as.integer(as.character(time))
                        mic1 = as.integer(as.character(mic1))
                        mic2 = as.integer(as.character(mic2))})
                foo = rbind(foo, temp)
                foo_row = nrow(foo)
                if (foo_row > 4000)
                    foo = foo[-seq_len(foo_row - 4000), ]
            }
            if (Sys.time() - last_plot > 0.015 && nrow(foo) > 1 ) {
                plot(mic1 ~ time, foo, type='l', ylim=c(0, 1024))
                lines(mic2 ~ time, foo, col='blue')
                #print(xyplot(mic1 + mic2 ~ time, foo, type='l', ylim=c(0, 1024)))
                last_plot = Sys.time()
            }
        }
        read.serialConnection(con)
        close(con)
}


do_read(100)

small_foo = subset(foo, time < 77500)
xyplot(mic1 + mic2 ~ time, small_foo, type='l')
