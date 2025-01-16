package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.ArrayList;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;

// Class that supports the writing out of OpenCV Mats to the local file system.
// This class has no notion of output levels the way that RobotLogCommon does.
// Users should rely on the logging level to control access to this class.
// For example:
// if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.v)) {
//    String fullFilename = pOutputFilenamePreamble + "_THR" + pFilenameSuffix + ".png";
//    DebugImageCommon.writeImage(fullFilename, thresholded);
//    RobotLogCommon.d(TAG, "Writing " + fullFilename);
// }
public class DebugImageCommon {

    private static final String TAG = DebugImageCommon.class.getSimpleName();

    public enum DrainOnClose {ALL, NONE}

    private static BiConsumer<String, String> logger;
    private static CompletableFuture<Void> imageWriterFuture;
    private static final LinkedBlockingQueue<Pair<String, Mat>> imageQueue = new LinkedBlockingQueue<>();
    private static final Lock imageWriterLock = new ReentrantLock();
    private static final Condition imageWriterCondition = imageWriterLock.newCondition();
    private static boolean imageWriterNotification = false; // protected by imageQueueLock
    private static boolean closeImageWriter = false; // protected by imageQueueLock
    private static DrainOnClose writeFilesOnClose; // protected by imageQueueLock

    // Use a Consumer<String> to accommodate System.out.println, the
    // FTC RobotLog, or the Android log.
    public static synchronized void initialize(BiConsumer<String, String> pLogger) {
        if (imageWriterFuture != null)
            return; // already initialized

        logger = pLogger;
        try {
            // Controlled startup of the DebugImageWriter thread.
            CountDownLatch imageWriterLatch = new CountDownLatch(1);
            imageWriterFuture = Threading.launchAsync(new DebugImageWriter(imageWriterLatch));
            imageWriterLatch.await(); // wait for the DebugImageWriter to start
            logger.accept(TAG, " DebugImageWriter up and running");
        } catch (Throwable throwable) {
            imageWriterFuture = null;
            logger.accept(TAG, " Error in DebugImageWriter initialization; writing of files is disabled");
        }
    }

    // Even though the BlockingQueue is thread-safe there's a warning about drainTo:
    // "A failure encountered while attempting to add elements to collection c
    // [during the drainTo] may result in elements being in neither, either or both
    // collections when the associated exception is thrown."
    public static synchronized void writeImage(String pFullFilename, Mat pImage) {
        if (imageWriterFuture == null)
            return;

        // Enqueue the image for writing out a file.
        imageWriterLock.lock();
        try {
            imageQueue.add(Pair.create(pFullFilename, pImage));
            imageWriterNotification = true;
            closeImageWriter = false;
            imageWriterCondition.signal();
        } finally {
            imageWriterLock.unlock();
        }
    }

    // Close DebugImageCommon and the associated DebugImageWriter thread.
    public static synchronized void close() {
        close(DrainOnClose.NONE); // default; exit immediately
    }

    // For testing, write out all queued files.
    public static synchronized void close(DrainOnClose pWriteFiles) {
        if (imageWriterFuture == null)
            return;

        // If the DebugImageWriter is hanging onto the lock for some reason or it
        // has already exited because of an InterruptedException, make sure it
        // can't be reused.
        if (!imageWriterLock.tryLock() || imageWriterFuture.isDone()) {
            imageWriterFuture = null;
            return;
        }

        // Signal the DebugImageWriter to exit.
        try {
            writeFilesOnClose = pWriteFiles;
            imageWriterNotification = true;
            closeImageWriter = true;
            imageWriterCondition.signal();
        } finally {
            imageWriterLock.unlock();
        }

        // Check if the DebugImageWriter has shut down cleanly.
        try {
            // Use a timeout value so that we never get hung up here.
            Threading.getFutureCompletion(imageWriterFuture, 100);

            // After a clean shutdown we can remove all traces of the DebugImageWriter.
            logger.accept(TAG, " DebugImageWriter thread completed succssfully");
        } catch (Throwable t) {
            // Our Threading.launchAsync method ensures that shutdownNow is called on
            // the executor in which the CompletableFuture is running.
            logger.accept(TAG, " Exception during shutdown of DebugImageWriter");
            logger.accept(TAG, " Error " + t);
        } finally {
            imageWriterFuture = null;
        }
    }

    // Separate thread that writes OpenCV images out to the local file system.
    private static class DebugImageWriter implements Callable<Void> {
        private final CountDownLatch countDownLatch;

        public DebugImageWriter(CountDownLatch pCountDownLatch) {
            countDownLatch = pCountDownLatch;
        }

        public Void call() {
            ArrayList<Pair<String, Mat>> drain = new ArrayList<>();

            // Use a countdown latch to signal that the CompletableFuture is started.
            countDownLatch.countDown();
            while (true) {
                try {
                    Objects.requireNonNull(imageWriterLock).lock();
                    while (!imageWriterNotification)
                        Objects.requireNonNull(imageWriterCondition).await();

                    imageWriterNotification = false;

                    // If there is a request to close the DebugImageWriter, give up immediately.
                    if (closeImageWriter) {
                        logger.accept(TAG, " Closing the DebugImageWriter with " + imageQueue.size() + " entries on the queue");
                        break;
                    }

                    // This is the normal path.
                    // Drain one or more entries from the queue to a local collection
                    // and then write them out *after* the lock is released.
                    Objects.requireNonNull(imageQueue).drainTo(drain); // must be protected by a lock
                } catch (InterruptedException iex) {  // await() can throw InterruptedException
                    break; // DebugImageWriter will exit
                } finally {
                    Objects.requireNonNull(imageWriterLock).unlock();
                }

                // We're *outside* the lock so more queue entries or a close request
                // may come in. But we won't see them until the following writes to
                // the file system have completed.
                if (writeFilesOnClose == DrainOnClose.ALL) {
                    for (Pair<String, Mat> oneImageEntry : drain) {
                        Imgcodecs.imwrite(oneImageEntry.first, oneImageEntry.second);
                    }
                }
                drain.clear();
            }

            return null;
        }
    }


}
