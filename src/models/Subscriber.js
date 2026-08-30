import mongoose from 'mongoose';

const SubscriberSchema = new mongoose.Schema({
  platformId: { type: String, required: true, unique: true },
  name: { type: String, default: 'عميل جديد' },
  step: { type: String, default: 'START' },
  phone: { type: String, default: 'غير متوفر' },
  lastInteraction: { type: Date, default: Date.now }
});

export default mongoose.models.Subscriber || mongoose.model('Subscriber', SubscriberSchema);
